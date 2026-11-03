#!/usr/bin/env python3
"""
vio_flight.py - Repeatable test flight for the VIO sim (iris_warehouse_vio.sdf)

Takes off, flies a fixed square in front of the spawn point with the nose
(and camera) kept facing the warehouse shelves, then lands. Every run flies
the same path so runs with and without injected faults can be compared.

Uses mavros_adapter.py (copied unchanged from ardupilot_autonomy). The
adapter's commands are fire-and-forget, so every step here waits until
telemetry shows it actually happened.

Positions are local ENU metres relative to the EKF origin (the spawn point);
yaw is degrees CCW from East, the adapter's convention.

Usage: python3 vio_flight.py [--alt 1.5] [--side 2.0] [--loops 1]
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node

from mavros_adapter import MavrosAdapter

ARRIVE_TOLERANCE_M = 0.25   # how close counts as "at the waypoint"
SETTLE_S = 2.0              # hover after each waypoint, lets VIO see a still moment
STEP_TIMEOUT_S = 90.0       # wall-clock limit per step (sim runs below real time)
RETRY_S = 3.0               # resend interval for arm / mode commands


class VioFlight:
    def __init__(self, node, adapter, alt, side, loops):
        self.node = node
        self.mav = adapter
        self.alt = alt
        self.side = side
        self.loops = loops
        self.yaw_deg = 0.0  # set at takeoff from the current heading

    def log(self, text):
        self.node.get_logger().info(text)

    def spin(self, secs):
        """Process callbacks for secs of wall time"""
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_until(self, what, condition, timeout=STEP_TIMEOUT_S, retry=None):
        """Spin until condition() is true. retry() is re-sent every RETRY_S."""
        start = time.monotonic()
        last_retry = start
        while not condition():
            if time.monotonic() - start > timeout:
                raise RuntimeError(f'timed out waiting for {what}')
            if retry and time.monotonic() - last_retry > RETRY_S:
                retry()
                last_retry = time.monotonic()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.log(f'ok: {what}')

    def distance_to(self, east, north, up):
        return math.sqrt((self.mav.odom_east - east) ** 2 +
                         (self.mav.odom_north - north) ** 2 +
                         (self.mav.odom_up - up) ** 2)

    def goto(self, east, north, up):
        self.log(f'goto E={east:.1f} N={north:.1f} U={up:.1f}')
        self.mav.goto_local(east, north, up, self.yaw_deg)
        self.wait_until(
            f'arrive at ({east:.1f}, {north:.1f}, {up:.1f})',
            lambda: self.distance_to(east, north, up) < ARRIVE_TOLERANCE_M,
            retry=lambda: self.mav.goto_local(east, north, up, self.yaw_deg))
        self.spin(SETTLE_S)

    def run(self):
        self.wait_until('MAVROS link to ArduPilot', lambda: self.mav.connected, timeout=60.0)

        self.mav.set_mode_guided()
        self.wait_until('GUIDED mode', lambda: self.mav.mode == 'GUIDED',
                        retry=self.mav.set_mode_guided)

        # Arming is refused until the EKF and GPS are ready, so keep retrying
        self.mav.arm()
        self.wait_until('armed', lambda: self.mav.armed, retry=self.mav.arm)

        # Hold the heading the drone has on the ground (camera facing the shelves)
        self.spin(0.5)
        self.yaw_deg = self.mav.get_telemetry()['local_position']['yaw_deg']
        home_e, home_n = self.mav.odom_east, self.mav.odom_north
        self.log(f'home E={home_e:.2f} N={home_n:.2f}, holding yaw {self.yaw_deg:.1f} deg')

        self.mav.takeoff(self.alt)
        self.wait_until(f'takeoff to {self.alt:.1f} m',
                        lambda: self.mav.odom_up > self.alt - ARRIVE_TOLERANCE_M)
        self.spin(SETTLE_S)

        # Square in front of the spawn point: forward along the nose, then left
        yaw = math.radians(self.yaw_deg)
        fwd = (math.cos(yaw), math.sin(yaw))     # unit vector along the nose (E, N)
        left = (-math.sin(yaw), math.cos(yaw))   # unit vector to the left (E, N)
        corners = [(0.0, 0.0), (self.side, 0.0), (self.side, self.side), (0.0, self.side), (0.0, 0.0)]

        for loop in range(1, self.loops + 1):
            self.log(f'square loop {loop}/{self.loops}')
            for f, l in corners:
                east = home_e + f * fwd[0] + l * left[0]
                north = home_n + f * fwd[1] + l * left[1]
                self.goto(east, north, self.alt)

        self.mav.land()
        self.wait_until('landed and disarmed', lambda: not self.mav.armed, timeout=120.0)
        self.log('flight complete')


def main():
    parser = argparse.ArgumentParser(description='Repeatable VIO test flight')
    parser.add_argument('--alt', type=float, default=1.5, help='flight height in metres')
    parser.add_argument('--side', type=float, default=2.0, help='side of the square in metres')
    parser.add_argument('--loops', type=int, default=1, help='times to fly the square')
    args = parser.parse_args()

    rclpy.init()
    node = Node('vio_flight')
    flight = VioFlight(node, MavrosAdapter(node), args.alt, args.side, args.loops)
    try:
        flight.run()
    except (RuntimeError, KeyboardInterrupt) as e:
        node.get_logger().error(f'flight aborted: {e}; landing')
        flight.mav.land()
        flight.spin(1.0)
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
