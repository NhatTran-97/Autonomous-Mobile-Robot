#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class StaticTransformsPublisher(Node):
    def __init__(self):
        super().__init__('static_transforms_publisher')
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_transforms()

    def publish_transforms(self):
        # # Transform cho RGBD Camera
        # camera_tf = TransformStamped()
        # camera_tf.header.stamp = self.get_clock().now().to_msg()
        # camera_tf.header.frame_id = 'base_link'
        # camera_tf.child_frame_id = 'camera_link'
        # camera_tf.transform.translation.x = 0.32
        # camera_tf.transform.translation.y = 0.0
        # camera_tf.transform.translation.z = 0.05
        # camera_tf.transform.rotation.x = 0.0
        # camera_tf.transform.rotation.y = 0.0
        # camera_tf.transform.rotation.z = 0.0
        # camera_tf.transform.rotation.w = 1.0

        # Transform cho LiDAR
        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = self.get_clock().now().to_msg()
        lidar_tf.header.frame_id = 'base_link'
        lidar_tf.child_frame_id = 'laser'
        lidar_tf.transform.translation.x = 0.285
        lidar_tf.transform.translation.y = 0.0
        lidar_tf.transform.translation.z = 0.0986
        lidar_tf.transform.rotation.x = 0.0
        lidar_tf.transform.rotation.y = 0.0
        lidar_tf.transform.rotation.z = 0.0
        lidar_tf.transform.rotation.w = 1.0

        # # Transform cho IMU
        # imu_tf = TransformStamped()
        # imu_tf.header.stamp = self.get_clock().now().to_msg()
        # imu_tf.header.frame_id = 'base_link'
        # imu_tf.child_frame_id = 'imu_link'
        # imu_tf.transform.translation.x = 0.205
        # imu_tf.transform.translation.y = 0.0
        # imu_tf.transform.translation.z = -0.018
        # imu_tf.transform.rotation.x = 0.0
        # imu_tf.transform.rotation.y = 0.0
        # imu_tf.transform.rotation.z = 0.0
        # imu_tf.transform.rotation.w = 1.0

        # Phát các transform (Static)
        self.tf_broadcaster.sendTransform([lidar_tf])
        self.get_logger().info('Static transforms have been published.')


def main():
    rclpy.init()
    node = StaticTransformsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
