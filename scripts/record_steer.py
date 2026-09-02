#!/usr/bin/env python3
"""
기준 운전자(IPGDriver)의 조향 각속도를 측정한다.

목적: MPC 의 조향 각속도 제약(model.ddelta_max, 기본 1.0 rad/s = 57.3 deg/s)이
실차 대비 지나치게 크다. 임의의 "현실적인 값"을 고르면 출처를 방어할 수 없으므로,
**같은 시뮬레이터의 기준 운전자가 같은 코스에서 실제로 쓰는 값**을 기준으로 삼는다.

MPC 없이 IPGDriver 로 주행하면서 /hmg_vehicle_state 의 steer 와 시각을 기록하고,
수치 미분으로 각속도를 얻는다.

✔ steer 는 **앞바퀴각 [rad]** 이다 (환산 불필요). 판별 근거: 최급코너
   R=37.7 m 의 Ackermann 조향각이 6.5° 인데 측정 최대가 9.6° 로 일치한다.
   핸들각이었다면 조향비(20~25)만큼 컸을 것이다.

사용 (host ROS + TMROS2 ws sourced, ROS_DOMAIN_ID 를 cm_node 와 맞출 것):
    python3 scripts/record_steer.py [out.npy]
  -> TruckMaker 를 IPGDriver 로 Start, 한 바퀴 주행 후 Ctrl-C
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from hmg_msgs.msg import VehicleState

OUT = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/vilab/CarMaker/mpc_host/ipgdriver_steer.npy"


class SteerRecorder(Node):
    def __init__(self):
        super().__init__("steer_recorder")
        self.sub = self.create_subscription(
            VehicleState, "/hmg_vehicle_state", self.cb, 50)
        self.rows = []      # [t, steer, v, x, y]
        self.t0 = None
        self.get_logger().info("Recording steer from /hmg_vehicle_state. Ctrl-C to save.")

    def cb(self, msg: VehicleState):
        t = msg.time.sec + msg.time.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        self.rows.append([t - self.t0, msg.steer, msg.v, msg.x, msg.y])
        if len(self.rows) % 500 == 0:
            self.get_logger().info(
                f"{len(self.rows)} samples, t={t-self.t0:.1f}s "
                f"steer={msg.steer:+.3f} v={msg.v:.1f}")


def report(a):
    """a: [t, steer, v, x, y]. 주행 중(v>1)인 구간만 사용."""
    t, st, v = a[:, 0], a[:, 1], a[:, 2]
    m = v > 1.0
    if m.sum() < 10:
        print("주행 구간이 너무 짧다 (v>1 샘플 부족)")
        return
    t, st = t[m], st[m]
    dt = np.diff(t)
    ok = dt > 1e-6
    rate = np.abs(np.diff(st)[ok] / dt[ok])          # 앞바퀴 조향 각속도 [rad/s]
    print(f"\n샘플 {m.sum()}개, 주행 {t[-1]-t[0]:.1f}s, 평균 dt {np.median(dt)*1000:.1f} ms\n")
    print(f"{'분위수':>8} {'앞바퀴 조향 각속도 [deg/s]':>26} {'rad/s':>10}")
    print("-" * 50)
    for q in (50, 90, 95, 99, 99.9, 100):
        r = np.percentile(rate, q)
        print(f"{q:7.1f}% {np.rad2deg(r):25.2f} {r:10.4f}")
    mx = rate.max()
    print(f"\n현재 MPC 제약: 1.0 rad/s = 57.3 deg/s  "
          f"(기준 운전자 최대의 {57.3/np.rad2deg(mx):.0f}배)")
    print(f"제안: DDELTA_MAX={round(2*mx, 2)}  (기준 운전자 최대 {np.rad2deg(mx):.2f} deg/s 의 2배)")


def main():
    rclpy.init()
    n = SteerRecorder()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    if len(n.rows) >= 50:
        a = np.array(n.rows)
        np.save(OUT, a)
        print(f"\nSaved {len(a)} samples -> {OUT}   (열: t, steer, v, x, y)")
        report(a)
    else:
        print(f"\n{len(n.rows)} 샘플뿐 — 저장 안 함. 시뮬이 돌고 있었나?")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
