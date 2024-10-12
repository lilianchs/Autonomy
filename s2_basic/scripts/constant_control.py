#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64, Bool, String

class Message(Node):
    def __init__(self) -> None:
        super().__init__("message")

        self.msg_pub = self.create_publisher(String, "/msg_channel", 10)
        self.msg_timer = self.create_timer(0.2, self.msg_callback)
        #self.msg_sub = self.create_subscription(String, "/msg_channel", 10)

    def msg_callback(self) -> None:
        msg = String()
        msg.data = "sending constant control"

        self.msg_pub.publish(msg)


if __name__ == "__main__":
    rclpy.init()
    node = Message()
    rclpy.spin(node)
    rclpy.shutdown()
