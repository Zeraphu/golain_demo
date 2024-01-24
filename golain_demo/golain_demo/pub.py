#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import HostData
import psutil

class MyNode(Node):
    def __init__(self):
        super().__init__("golain_pub")
        self.golain_pub_ = self.create_publisher(HostData,"golain/data",10)
        self.create_timer(0.5,self.timer_callbacks)
        self.get_logger().info("Pub Started")

    def timer_callbacks(self):
        msg = HostData()
        msg.ramperc = psutil.virtual_memory()[2]
        msg.ramuse = psutil.virtual_memory()[3]/1000000000
        self.golain_pub_.publish(msg)
        self.get_logger().info("RAM % Used: " + str(psutil.virtual_memory()[2]))
        self.get_logger().info("RAM Used (GB): " + str(psutil.virtual_memory()[3]/1000000000))

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()