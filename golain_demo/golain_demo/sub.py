#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import HostData
from golain_client import GolainClient
from shadow_pb2 import Shadow, SSHSetting
from BatteryData_pb2 import BatteryData


import paho.mqtt.client as mqtt
from google._upb._message import MessageMeta
import json

class GolainClient:
    def __init__(self, shadow_type: MessageMeta):
        self._shadowType = shadow_type
        self.Shadow = self._shadowType()
        self.dataPointMap = {}
        # load the connection settings from the json file
        try:
            with open('connection_settings.json') as f:
                settings = json.load(f)
                client_id = settings['device_name']
                root_topic = settings['root_topic']
                self.client_id = client_id
                self.root_topic = root_topic
                self.host = settings['host']
                self._SHADOW_TOPIC_R = f'{root_topic}{client_id}/device-shadow/r'
                self._SHADOW_TOPIC_W = f'{root_topic}{client_id}/device-shadow/u'
                self._DATA_TOPIC = f'{root_topic}{client_id}/device-data/'
                # convert the port to an integer
                self.port = int(settings['port'])
        except FileNotFoundError:
            print("Connection settings file not found!")
            exit(1)
      
    def registerShadowCallback(self, callback):
      self.callback = callback

    def publishData(self, message, name: str):
      self._publish(f'{self._DATA_TOPIC}{name}', message.SerializeToString())

    def _publish(self, topic, payload):
        self.client.publish(topic, payload)

    def loop_blocking(self):
        self.client.loop_forever()

    def _on_message(self, client, userdata, message):
        print(f'message received on: {message.topic}')
        if message.topic == self._SHADOW_TOPIC_R:
            self.Shadow.ParseFromString(message.payload)
            print(f'Updated shadow: {self.Shadow}')
            self.callback(self.Shadow)

    def _subscribe(self, topic, callback):
        self.client.subscribe(topic)
        self.client.on_message = callback

    def _on_subscribe(self, client, userdata, mid, granted_qos):
        print(f'subsription successful: {mid}, {granted_qos}, {userdata}')

    def updateShadow(self):
        self._publish(self._SHADOW_TOPIC_W, self.Shadow.SerializeToString())

    def connect(self):
        # Create a new MQTT client
        self.client = mqtt.Client(client_id=self.client_id, reconnect_on_failure=True, clean_session=False)
        # lambda function to handle the message
        self.client.on_message = self._on_message
        try:
            self.client.tls_set(ca_certs="root_ca_cert.pem", certfile="device_cert.pem", keyfile="device_private_key.pem")
            self.client.tls_insecure_set(True)
        except FileNotFoundError:
            print("TLS certificates not found!")
            exit(1)
        try:
            self.client.on_subscribe = self._on_subscribe
            self.client.connect(self.host, self.port)
            # device shadow
            self.client.subscribe(self.root_topic + self.client_id + "/device-shadow/r", 1)
            # device ota
            self.client.subscribe(self.root_topic + self.client_id + "/device-ota/+", 1)
            self.client.loop_start()
        except Exception as e:
            print(e)
            print("Connection failed!")
            exit(1)

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
