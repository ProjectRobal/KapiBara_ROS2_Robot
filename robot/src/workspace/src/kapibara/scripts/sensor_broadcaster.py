#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from kapibara.initialize import initialize_devices

class SensorBroadcaster(Node):

    def __init__(self):
        super().__init__('SensorBroadcaster')
        self.init_devices()

    def init_devices(self):
        self.device_list=initialize_devices(self)
        for dev in self.device_list:
            dev.start()


def main(args=None):
    rclpy.init(args=args)

    sensors = SensorBroadcaster()

    rclpy.spin(sensors)

    sensors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()