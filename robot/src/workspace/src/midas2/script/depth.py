#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge

from midas2.midas import midasDepthEstimator


import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage,Image
from sensor_msgs.msg import PointCloud2,PointField

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
        
        self.depth_point_publisher = self.create_publisher(PointCloud2,'/midas/points',10)
        
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
        
        gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        gray_img = cv2.resize(gray_img,(16,16))
        
        gray_img = cv2.normalize(gray_img, None, 0.0, 1.0, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        
        width,height = gray_img.shape
        
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header.frame_id = "camera_optical"
        
        point_x = PointField()
        
        point_x.name = 'x'
        point_x.offset = 0
        point_x.datatype = PointField.FLOAT32
        point_x.count = 1
        
        point_y = PointField()
        
        point_y.name = 'y'
        point_y.offset = 4
        point_y.datatype = PointField.FLOAT32
        point_y.count = 1
        
        point_z = PointField()
        
        point_z.name = 'z'
        point_z.offset = 8
        point_z.datatype = PointField.FLOAT32
        point_z.count = 1
        
        point_i = PointField()
        
        point_i.name = 'intensity'
        point_i.offset = 12
        point_i.datatype = PointField.FLOAT32
        point_i.count = 1

        # Define the point fields (attributes)        
        fields =[point_x,
                point_y,
                point_z,
                point_i,
                ]

        pointcloud_msg.fields = fields

        pointcloud_msg.height = height
        pointcloud_msg.width = width
        
        # Float occupies 4 bytes. Each point then carries 16 bytes.
        pointcloud_msg.point_step = len(fields) * 4 
        
        total_num_of_points = pointcloud_msg.height * pointcloud_msg.width
        pointcloud_msg.row_step = pointcloud_msg.point_step * total_num_of_points
        pointcloud_msg.is_dense = True
        
        scale = 1.0
        
        data = np.zeros((height,width,len(fields)),dtype=np.float32)
        
        for y in range(height):
            for x in range(width):
                pixel = gray_img[x,y]
                
                distance = 1.0 / ( ((pixel)*scale) + 10e-6)
                
                data[y][x][0] = (x - width/2)/width
                data[y][x][1] = (y - height/2)/height
                data[y][x][2] = distance
                data[y][x][3] = pixel/255.0
        
        pointcloud_msg.data = data.tobytes()
                
        self.depth_point_publisher.publish(pointcloud_msg)
                
        
        # estimate 
        
        

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