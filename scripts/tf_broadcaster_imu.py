#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf2_ros
from geometry_msgs.msg import TransformStamped


class IMUTFBroadcaster(Node):

    def __init__(self):
        super().__init__('imu_tf_broadcaster')

        self.br = tf2_ros.TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.handle_imu,
            10
        )

    def handle_imu(self, msg):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "plane"
        t.child_frame_id = "imu_link"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation = msg.orientation

        self.br.sendTransform(t)


def main(args=None):

    rclpy.init(args=args)

    node = IMUTFBroadcaster()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()