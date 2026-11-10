#!/usr/bin/env python3
"""
record_run.py - Records one VIO run for scoring afterwards

Start it after OpenVINS and before the flight, stop it with Ctrl-C after
landing. Saves to runs/eval/<name>/:

  estimate.tum       OpenVINS pose (/ov_msckf/poseimu)
  ground_truth.tum   Gazebo pose (/vio/ground_truth/odom)
  estimate_cov.csv   OpenVINS position and orientation variances
  events.csv         MAVROS armed / mode changes, for finding takeoff and landing

The .tum files are "t x y z qx qy qz qw". Times are sim time, read from the
message headers. MAVROS stamps with the wall clock, so events are tagged
with the latest ground truth time instead.

Usage: python3 record_run.py [--name square_01]
"""

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry

RUNS_DIR = Path(__file__).resolve().parents[3] / 'runs' / 'eval'
STATUS_PERIOD_S = 5.0


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def tum_line(t, pose):
    p, q = pose.position, pose.orientation
    return f'{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.8f} {q.y:.8f} {q.z:.8f} {q.w:.8f}\n'


class RunRecorder(Node):
    def __init__(self, out_dir):
        super().__init__('record_run')
        self.est_file = open(out_dir / 'estimate.tum', 'w')
        self.gt_file = open(out_dir / 'ground_truth.tum', 'w')
        self.cov_file = open(out_dir / 'estimate_cov.csv', 'w')
        self.events_file = open(out_dir / 'events.csv', 'w')
        self.cov_file.write('t,var_x,var_y,var_z,var_rx,var_ry,var_rz\n')
        self.events_file.write('t,armed,mode\n')

        self.num_est = 0
        self.num_gt = 0
        self.sim_now = 0.0
        self.last_state = None

        self.create_subscription(PoseWithCovarianceStamped, '/ov_msckf/poseimu', self.on_estimate, 50)
        self.create_subscription(Odometry, '/vio/ground_truth/odom', self.on_ground_truth, 50)

        # Matches the MAVROS publisher, same as mavros_adapter.py
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(State, '/mavros/state', self.on_state, state_qos)

        self.create_timer(STATUS_PERIOD_S, self.on_status)
        self.get_logger().info(f'recording to {out_dir}')

    def on_estimate(self, msg):
        t = stamp_to_sec(msg.header.stamp)
        self.est_file.write(tum_line(t, msg.pose.pose))
        c = msg.pose.covariance
        self.cov_file.write(f'{t:.6f},' + ','.join(f'{c[7 * i]:.10e}' for i in range(6)) + '\n')
        self.num_est += 1

    def on_ground_truth(self, msg):
        self.sim_now = stamp_to_sec(msg.header.stamp)
        self.gt_file.write(tum_line(self.sim_now, msg.pose.pose))
        self.num_gt += 1

    def on_state(self, msg):
        state = (msg.armed, msg.mode)
        if state != self.last_state:
            self.events_file.write(f'{self.sim_now:.6f},{int(msg.armed)},{msg.mode}\n')
            self.last_state = state
            self.get_logger().info(f'armed={msg.armed} mode={msg.mode}')

    def on_status(self):
        self.get_logger().info(f'estimate={self.num_est} ground_truth={self.num_gt} sim_t={self.sim_now:.1f}')
        self.flush()

    def flush(self):
        for f in (self.est_file, self.gt_file, self.cov_file, self.events_file):
            f.flush()

    def close(self):
        for f in (self.est_file, self.gt_file, self.cov_file, self.events_file):
            f.close()


def main():
    parser = argparse.ArgumentParser(description='Record one VIO run')
    parser.add_argument('--name', default=time.strftime('run_%Y%m%d_%H%M%S'), help='folder name under runs/eval')
    args = parser.parse_args()

    out_dir = RUNS_DIR / args.name
    if out_dir.exists():
        sys.exit(f'{out_dir} already exists, pick another --name')
    out_dir.mkdir(parents=True)

    rclpy.init()
    node = RunRecorder(out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f'saved {node.num_est} estimate and {node.num_gt} ground truth samples to {out_dir}')
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
