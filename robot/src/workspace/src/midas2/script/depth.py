#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge

import numpy as np
from midas2.midas import midasDepthEstimator

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage,Image

from rcl_interfaces.msg import ParameterDescriptor

import numpy as np
import base64

from timeit import default_timer as timer

class MidasNode(Node):

    def __init__(self):
        super().__init__('MiDaS2')
        
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed',descriptor=ParameterDescriptor(name='camera_topic', type=0, description="A topic to subscribe for compressed image"))
        
        # self.declare_parameter('model_name', 'DeepID',descriptor=ParameterDescriptor(name="model_name", type=0, description="A model used for face analyze"))
        # self.declare_parameter('detector_backend', 'yunet',descriptor=ParameterDescriptor(name="detector_backend", type=0, description="A backend used for face detection on image"))
        
        self.bridge = CvBridge()
        
        # initialize model
        self.get_logger().info('Initializing MiDas2...')
        
        self.midas = midasDepthEstimator()
        
        self.get_logger().info('Model initialized!')
        
        self.depth_publisher = self.create_publisher(CompressedImage, '/midas/depth/compressed', 10)
        self.depth_publisher_raw = self.create_publisher(Image, '/midas/depth', 10)
        
        self.subscription = self.create_subscription(
            CompressedImage,
            self.get_parameter('camera_topic').get_parameter_value().string_value,
            self.camera_listener,
            10)
        self.subscription  # prevent unused variable warning

    def camera_listener(self, msg:CompressedImage):
        self.get_logger().debug('I got image with format: %s' % msg.format)
        
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        
        start = timer()
        
        colorDepth = self.midas.estimateDepth(image)
        
        self.get_logger().info('Estimation time: %f s' % ( timer() - start ))
        
        self.depth_publisher.publish(self.bridge.cv2_to_compressed_imgmsg(colorDepth))
        self.depth_publisher_raw.publish(self.bridge.cv2_to_imgmsg(colorDepth))
        
        

def main(args=None):
    rclpy.init(args=args)

    midas = MidasNode()

    rclpy.spin(midas)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    midas.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()