#!/usr/bin/env python3
"""
MPC ROS RELAY (rclpy / DDS only, NO acados/torch).

Bridges:  /hmg_vehicle_state  --socket-->  mpc_solver_server.py  --socket-->  /hmg_ctrl

Runs with the SYSTEM python (host ROS), so the acados/fast-DDS conflict cannot
occur here (this process never loads acados). The heavy MPC lives in the
separate solver server.

Env: MPC_SOCK (unix socket to the solver), CTRL_HZ.
"""
import os
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from hmg_msgs.msg import VehicleState, TestMsgs

SOCK    = os.environ.get("MPC_SOCK", "/tmp/mpc_sock")
CTRL_HZ = float(os.environ.get("CTRL_HZ", "10.0"))

REQ  = struct.Struct("dddd")   # x, y, yaw, v
RESP = struct.Struct("dddd")   # gas, brake, steering, gear


class Relay(Node):
    def __init__(self, conn):
        super().__init__("mpc_relay")
        self.conn = conn
        self.sub = self.create_subscription(
            VehicleState, "/hmg_vehicle_state", self.cb, 10)
        self.pub = self.create_publisher(TestMsgs, "/hmg_ctrl", 10)
        self.msg_count = 0
        self.every = max(1, int(round(200.0 / CTRL_HZ)))
        self.n_pub = 0
        self.get_logger().info("MPC relay up. Bridging /hmg_vehicle_state <-> solver <-> /hmg_ctrl")

    def cb(self, msg: VehicleState):
        self.msg_count += 1
        if self.msg_count % self.every:
            return
        try:
            self.conn.sendall(REQ.pack(msg.x, msg.y, msg.yaw, msg.v))
            data = self.conn.recv(RESP.size)
            if len(data) < RESP.size:
                self.get_logger().error("solver closed connection; shutting down")
                rclpy.shutdown()
                return
            gas, brake, steering, gear = RESP.unpack(data)
        except (OSError, struct.error) as e:
            self.get_logger().error(f"socket error: {e}")
            return

        m = TestMsgs()
        m.gas = float(gas)
        m.brake = float(brake)
        m.steering = float(steering)
        m.gear = int(gear)
        self.pub.publish(m)
        self.n_pub += 1
        if self.n_pub % 20 == 0:
            self.get_logger().info(
                f"x={msg.x:.1f} y={msg.y:.1f} v={msg.v:.2f} -> "
                f"gas={gas:.2f} brake={brake:.2f} steer={steering:+.2f}")


def main():
    # wait for the solver server's socket (it takes ~40 s to build)
    conn = None
    for _ in range(180):
        if os.path.exists(SOCK):
            try:
                conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                conn.connect(SOCK)
                conn.settimeout(5.0)
                break
            except OSError:
                conn = None
        time.sleep(1)
    if conn is None:
        print("[relay] solver socket never appeared; exiting")
        return
    print("[relay] connected to solver", flush=True)

    rclpy.init()
    node = Relay(conn)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    conn.close()


if __name__ == "__main__":
    main()
