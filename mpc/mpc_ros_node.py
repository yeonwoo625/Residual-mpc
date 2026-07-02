#!/usr/bin/env python3
"""
Residual MPC ROS2 node for TruckMaker.

  /hmg_vehicle_state (hmg_msgs/VehicleState)  --subscribe-->  Residual MPC
        -> control [D, delta] -> /hmg_ctrl (hmg_msgs/TestMsgs)

Control mapping (identical to CARLA _D_to_carla_control):
  D >= 0 : gas = D,   brake = 0
  D <  0 : gas = 0,   brake = -D
  delta  : DrivMan.Steering.Ang [rad]   (see STEER_RATIO note)
  gear   : 1 (Drive)

Reference path: straight line from the truck's first observed pose (kappa=0).
This is the simplest reference for end-to-end loop validation;
replace make_straight_waypoints() with the road centerline later.
"""
import os
import numpy as np
import torch
torch.set_num_threads(1)   # avoid torch thread/rclpy-callback conflicts (segfaults)
import rclpy
from rclpy.node import Node
from hmg_msgs.msg import VehicleState, TestMsgs

from reference_utils import (
    compute_path_spline,
    compute_kappa_spline_for_acados,
    cartesian_to_frenet,
)
from mpc_solver_residual import MPCSolverResidual

# ---- tunables ----
TARGET_SPEED = float(os.environ.get("TARGET_SPEED", "4.0"))   # [m/s] low for curve
Tf, N        = 2.0, 20
CTRL_HZ      = 10.0
MODEL_PATH   = os.environ.get(
    "RESIDUAL_MODEL_PATH",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "residual_model.pt"))
# DrivMan.Steering.Ang is the STEERING-WHEEL angle; MPC delta is the road-wheel
# angle. They differ by the steering ratio. Start at 1.0 and calibrate against
# the truck once lateral tracking is exercised (straight path keeps delta ~0).
STEER_RATIO  = float(os.environ.get("STEER_RATIO", "10.0"))
PATH_LENGTH  = 500.0        # [m] straight reference length
# If set to a .npy of (N,2) waypoints, follow that recorded road (curved)
# instead of a straight line. Built eagerly so control starts immediately.
REFERENCE_PATH = os.environ.get("REFERENCE_PATH", "")


def make_straight_waypoints(x0, y0, yaw0, length=PATH_LENGTH, ds=2.0):
    d = np.arange(0.0, length, ds)
    xs = x0 + d * np.cos(yaw0)
    ys = y0 + d * np.sin(yaw0)
    return np.stack([xs, ys], axis=1)


class ResidualMPCNode(Node):
    def __init__(self):
        super().__init__("residual_mpc")
        self.sub = self.create_subscription(
            VehicleState, "/hmg_vehicle_state", self.state_cb, 10)
        self.pub = self.create_publisher(TestMsgs, "/hmg_ctrl", 10)

        self.state = None        # (x, y, yaw, v)
        self.mpc = None
        self.path_info = None
        self.D_prev = 0.0
        self.delta_prev = 0.0
        self.n_solve = 0
        self.msg_count = 0
        # /hmg_vehicle_state is ~200 Hz; solve every Nth msg to run near CTRL_HZ.
        self.ctrl_every = max(1, int(round(200.0 / CTRL_HZ)))

        # Eagerly build from a recorded reference path so control starts the
        # instant state arrives (codegen happens now, not mid-drive).
        if REFERENCE_PATH and os.path.exists(REFERENCE_PATH):
            wpts = np.load(REFERENCE_PATH)
            self.get_logger().info(
                f"Reference path: {len(wpts)} waypoints from {REFERENCE_PATH}")
            self._build_mpc(wpts)

        # Control runs *inside* the subscription callback (no timer). When the
        # publisher stops, the callback simply stops firing, so we never solve
        # stale state concurrently with DDS publisher teardown (which segfaults).
        self.get_logger().info("Residual MPC node up. Waiting for /hmg_vehicle_state ...")

    def state_cb(self, msg: VehicleState):
        self.state = (msg.x, msg.y, msg.yaw, msg.v)
        if self.mpc is None:
            if not (REFERENCE_PATH and os.path.exists(REFERENCE_PATH)):
                self._build_mpc(make_straight_waypoints(msg.x, msg.y, msg.yaw))
            return
        self.msg_count += 1
        if self.msg_count % self.ctrl_every == 0:
            self.control_step()

    def _build_mpc(self, wpts):
        self.path_info = compute_path_spline(np.asarray(wpts, dtype=float))
        kmax = float(np.max(np.abs(self.path_info["kappa"])))
        self.get_logger().info(
            f"Building MPC. Path len={self.path_info['s_total']:.1f}m "
            f"max_kappa={kmax:.4f}, target {TARGET_SPEED} m/s (codegen ~40s) ...")
        coeffs, knots = compute_kappa_spline_for_acados(
            self.path_info["dense_s"], self.path_info["kappa"], degree=3)
        self.mpc = MPCSolverResidual(
            coeffs=coeffs, knots=knots, model_path=MODEL_PATH,
            Tf=Tf, N=N, target_speed=TARGET_SPEED,
            global_path_length=self.path_info["s_total"],
        )
        self.mpc.update_path(self.path_info["dense_s"], self.path_info["kappa"])
        self.get_logger().info("MPC solver ready. Controlling.")

    def control_step(self):
        if self.mpc is None or self.state is None:
            return
        X, Y, yaw, v = self.state
        s, n, alpha = cartesian_to_frenet(X, Y, yaw, self.path_info)

        if self.n_solve == 0:
            self.get_logger().info(
                f"first state: x={X:.1f} y={Y:.1f} -> s={s:.1f} n={n:.2f} "
                f"a={np.rad2deg(alpha):.1f} v={v:.2f}")

        # guard acados against off-path / non-finite states (would segfault)
        x0 = np.array([s, n, alpha, v, self.D_prev, self.delta_prev])
        if (not np.all(np.isfinite(x0))) or abs(n) > 8.0 \
                or s < 0.0 or s >= self.path_info["s_total"] - 5.0:
            if self.n_solve % 10 == 0:
                self.get_logger().warn(
                    f"skip solve (out of bounds): s={s:.1f} n={n:.2f} v={v:.2f}")
            self.n_solve += 1
            self._publish(0.0, 0.0)
            return
        try:
            D, delta, status = self.mpc.solve(x0)
        except Exception as e:
            self.get_logger().error(f"MPC solve raised: {e}")
            return
        if status not in (0, 2):
            self.get_logger().warn(f"solver status={status}")

        self.D_prev, self.delta_prev = D, delta
        self._publish(D, delta)

        self.n_solve += 1
        if self.n_solve % 10 == 0:
            self.get_logger().info(
                f"s={s:6.1f} n={n:+.2f} a={np.rad2deg(alpha):+.1f} "
                f"v={v:.2f}/{TARGET_SPEED} D={D:+.3f} delta={np.rad2deg(delta):+.1f}")

    def _publish(self, D, delta):
        m = TestMsgs()
        if D >= 0:
            m.gas = float(np.clip(D, 0.0, 1.0)); m.brake = 0.0
        else:
            m.gas = 0.0; m.brake = float(np.clip(-D, 0.0, 1.0))
        m.steering = float(delta * STEER_RATIO)
        m.gear = 1  # Drive
        self.pub.publish(m)


def main():
    rclpy.init()
    node = ResidualMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
