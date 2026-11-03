#!/usr/bin/env python3
"""
gz_imu_bridge.py - Gazebo Harmonic IMU -> ROS 2 sensor_msgs/Imu

The installed ros_gz_bridge is built for Gazebo Garden and corrupts IMU
messages coming from Harmonic (acceleration arrives as zeros). This node
reads the IMU with Harmonic's own Python bindings and republishes it.

Usage: python3 gz_imu_bridge.py [gz_topic] [ros_topic]
       defaults: /vio/imu  /vio/imu
"""

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from gz.msgs10.imu_pb2 import IMU
from gz.transport13 import Node as GzNode


class GzImuBridge(Node):
    def __init__(self, gz_topic, ros_topic):
        super().__init__('gz_imu_bridge')
        self.pub = self.create_publisher(Imu, ros_topic, 100)
        self.gz_node = GzNode()
        if not self.gz_node.subscribe(IMU, gz_topic, self.on_imu):
            raise RuntimeError(f'could not subscribe to gz topic {gz_topic}')
        self.get_logger().info(f'bridging gz {gz_topic} -> ros {ros_topic}')

    def on_imu(self, gz_msg):
        msg = Imu()

        # Sim-time stamp and frame name straight from Gazebo
        msg.header.stamp.sec = gz_msg.header.stamp.sec
        msg.header.stamp.nanosec = gz_msg.header.stamp.nsec
        for item in gz_msg.header.data:
            if item.key == 'frame_id' and item.value:
                msg.header.frame_id = item.value[0]

        msg.orientation.x = gz_msg.orientation.x
        msg.orientation.y = gz_msg.orientation.y
        msg.orientation.z = gz_msg.orientation.z
        msg.orientation.w = gz_msg.orientation.w

        msg.angular_velocity.x = gz_msg.angular_velocity.x
        msg.angular_velocity.y = gz_msg.angular_velocity.y
        msg.angular_velocity.z = gz_msg.angular_velocity.z

        msg.linear_acceleration.x = gz_msg.linear_acceleration.x
        msg.linear_acceleration.y = gz_msg.linear_acceleration.y
        msg.linear_acceleration.z = gz_msg.linear_acceleration.z

        # Covariances are only copied when Gazebo fills them (3x3 = 9 values)
        if len(gz_msg.orientation_covariance.data) == 9:
            msg.orientation_covariance = list(gz_msg.orientation_covariance.data)
        if len(gz_msg.angular_velocity_covariance.data) == 9:
            msg.angular_velocity_covariance = list(gz_msg.angular_velocity_covariance.data)
        if len(gz_msg.linear_acceleration_covariance.data) == 9:
            msg.linear_acceleration_covariance = list(gz_msg.linear_acceleration_covariance.data)

        self.pub.publish(msg)


def main():
    gz_topic = sys.argv[1] if len(sys.argv) > 1 else '/vio/imu'
    ros_topic = sys.argv[2] if len(sys.argv) > 2 else '/vio/imu'

    rclpy.init()
    node = GzImuBridge(gz_topic, ros_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
