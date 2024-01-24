#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import HostData
from golain_client import GolainClient
from shadow_pb2 import Shadow, SSHSetting
from BatteryData_pb2 import BatteryData


def shadowCallback(shadow: Shadow):
    print(f'Updated shadow in callback: {shadow}')

class MyNode(Node):
    def __init__(self):
        self.Client = GolainClient(Shadow)
        super().__init__("golain_sub")
        self.golain_sub_ = self.create_subscription(HostData,"golain/data",self.data_callback,10)
        self.get_logger().info("Sub Started")
        self.Client.registerShadowCallback(shadowCallback)
        self.Client.connect()

    def data_callback(self, msg: HostData):
        self.get_logger().info('Ramperc is: ' + str(msg.ramperc))
        self.get_logger().info('Ramuse is: ' + str(msg.ramuse))
        toSend = BatteryData()
        toSend.voltage = msg.ramperc
        self.Client.publishData(toSend, "Battery Data")
        self.Client.Shadow.sshSetting = 1
        self.Client.updateShadow()

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()