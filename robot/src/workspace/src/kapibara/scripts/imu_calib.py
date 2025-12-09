#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from kapibara.initialize import initialize_devices
            

def main(args=None):
    rclpy.init(args=args)

    dumy_node=Node("imu_calibration")

    initialize_devices(dumy_node,True)

    rclpy.spin(dumy_node)

    dumy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

