#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')

        # Cho phép đổi tên topic bằng tham số
        self.declare_parameter('twist_topic_in', '/cmd_vel')
        self.declare_parameter('twiststamped_topic_out', '/diff_cont/cmd_vel')

        twist_topic_in = self.get_parameter('twist_topic_in').get_parameter_value().string_value
        twiststamped_topic_out = self.get_parameter('twiststamped_topic_out').get_parameter_value().string_value

        # Sub Twist
        self.sub = self.create_subscription(
            Twist,
            twist_topic_in,
            self.cmd_vel_callback,
            10
        )

        # Pub TwistStamped
        self.pub = self.create_publisher(
            TwistStamped,
            twiststamped_topic_out,
            10
        )

        self.get_logger().info(
            f'Subscribing Twist from [{twist_topic_in}] '
            f'and publishing TwistStamped to [{twiststamped_topic_out}]'
        )

    def cmd_vel_callback(self, msg: Twist):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist = msg
        self.pub.publish(ts)


def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

