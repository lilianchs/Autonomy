#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import String, Int64, Bool
from geometry_msgs.msg import Twist


class ConstantControl(Node):
    def __init__(self) -> None:
	    # initialize base class (must happen before everything else)
        super().__init__("constant_control")
				
	    # create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        self.cc_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # create a timer with: self.create_timer(<second>, <callback>)
        self.cc_timer = self.create_timer(0.2, self.cc_callback)

	    # create subscription with: self.create_subscription(<msg type>, <topic>, <callback>, <qos>)
        self.kill_sub = self.create_subscription(Bool, "/kill", self.kill_callback, 10)

    def cc_callback(self) -> None:
        msg = Twist()
        msg.linear.x = 1.1
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # publish heartbeat counter
        self.cc_pub.publish(msg)

    def kill_callback(self, msg: Bool) -> None:
        """
        Sensor health callback triggered by subscription
        """
        if msg.data:
            self.cc_timer.cancel()
            new_msg = Twist()
            new_msg.linear.x = 0.0
            new_msg.linear.y = 0.0
            new_msg.linear.z = 0.0
            new_msg.angular.x = 0.0
            new_msg.angular.y = 0.0
            new_msg.angular.z = 0.0
            self.cc_pub.publish(new_msg)


if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = ConstantControl()  # instantiate the heartbeat node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context

