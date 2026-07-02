#!/usr/bin/env python3
"""
Record the reference path for the MPC by capturing /hmg_vehicle_state (x,y)
while the truck drives the (curved) road under IPGDriver. Downsamples to one
waypoint per ~MIN_DS metres so the curvature spline stays smooth.

Usage (host ROS + TMROS2 ws sourced):
    python3 record_path.py [out.npy]
Drive the truck (IPGDriver), then Ctrl-C to save.
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from hmg_msgs.msg import VehicleState

OUT = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/vilab/CarMaker/mpc_host/ref_waypoints.npy"
MIN_DS = 2.0   # metres between recorded waypoints


class Recorder(Node):
    def __init__(self):
        super().__init__("path_recorder")
        self.sub = self.create_subscription(
            VehicleState, "/hmg_vehicle_state", self.cb, 50)
        self.pts = []
        self.last = None
        self.get_logger().info("Recording (x,y) from /hmg_vehicle_state. Ctrl-C to save.")

    def cb(self, msg: VehicleState):
        p = np.array([msg.x, msg.y])
        if self.last is None or np.linalg.norm(p - self.last) >= MIN_DS:
            self.pts.append(p)
            self.last = p
            if len(self.pts) % 10 == 0:
                self.get_logger().info(
                    f"{len(self.pts)} wpts, last=({msg.x:.1f},{msg.y:.1f}) v={msg.v:.1f}")


def main():
    rclpy.init()
    n = Recorder()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    if len(n.pts) >= 4:
        arr = np.array(n.pts)
        np.save(OUT, arr)
        print(f"\nSaved {len(arr)} waypoints to {OUT}")
        print(f"  start=({arr[0,0]:.1f},{arr[0,1]:.1f})  end=({arr[-1,0]:.1f},{arr[-1,1]:.1f})")
    else:
        print(f"\nOnly {len(n.pts)} waypoints — not saved (need >=4). Was the sim running?")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
