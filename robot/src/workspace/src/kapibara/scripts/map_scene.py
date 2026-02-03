#!/usr/bin/env python3

from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage,Image

from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry,OccupancyGrid


import cv2
from cv_bridge import CvBridge

import os

import numpy as np

@dataclass
class Checkpoint:
    image:np.ndarray
    x:float
    y:float
    yaw:float

class SceneMapper(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        
        self.declare_parameter("output_path")
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/KapiBara/odom',
            self.odom_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/KapiBara/map',
            self.map_callaback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/KapiBara/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.output_path = self.get_parameter("output_path").get_parameter_value().string_value
        
        self.x = 0.0
        self.y = 0.0
        
        self.yaw = 0.0
        
        self.target_yaw = 0.0
        
        # map coordinates
        self.map_x = 0
        self.map_y = 0
        self.resolution = 0.05
                                        
        self.map = None
        self.image = None
        
        self.checkpoints:list[Checkpoint] = []
        
        self.bridge = CvBridge()
        
        # it will holds data of collected maps with associated images
        self.timestamps = []
                
    def add_checkpoint(self):
        checkpoint = Checkpoint(
            image=self.image.copy(),
            x=self.x,
            y=self.y,
            yaw=self.yaw
        )
        self.checkpoints.append(checkpoint)
        self.get_logger().info(f'Checkpoint added at position: {self.x} {self.y} {self.yaw}')
        
    def memorize(self):
        '''
        Docstring for memorize
        
        A function that takes collected checkpoints
        and use them to train neural network to memorize
        map.
        '''
        pass
    
    def quaternion_to_yaw(self,q):
        """
        Convert a ROS2 geometry_msgs.msg.Quaternion to yaw (rad).
        """
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw
    
    def position_to_map_coords(self, x:float, y:float):
        '''
        Convert world coordinates to map coordinates
        '''
        if self.map is None:
            return None, None
        
        map_x = int((x - self.map_x) / self.resolution)
        map_y = int((y - self.map_y) / self.resolution)
        
        if map_x < 0 or map_x >= self.map.shape[1] or map_y < 0 or map_y >= self.map.shape[0]:
            return None, None
        
        return map_x, map_y
    
    def odom_callback(self,msg: Odometry):
                
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # set point as visited
        map_x, map_y = self.position_to_visited_coords(self.x, self.y)
        if map_x is not None and map_y is not None:
            self.visited_points[map_y, map_x] = 1.0
        
        self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
                        
        self.get_logger().info(f'Robot position: {self.x} {self.y} {self.yaw}')
        self.wait_for_odom = False
                
    def map_callaback(self, msg: OccupancyGrid):
        self.get_logger().info('Got map!')
        self.map = np.array(msg.data,dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map = self.map.astype(np.float32) / 100.0
        
        self.map_x = msg.info.origin.position.x
        self.map_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        
    def image_callback(self,msg:Image):
        
        self.get_logger().info('Got image with format: %s' % msg.encoding)
        
        if self.map is not None or self.wait_for_odom:
            self.get_logger().info('Waitting for map and odometry data')
            return
        
        self.image = self.bridge.imgmsg_to_cv2(msg)
        
        grayscale = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        
        img = cv2.resize(grayscale,(224,224))
        
        map = np.array(self.map)
        
        m_x,m_y = self.position_to_map_coords(self.x,self.y)
        
        map[m_x][m_y] = 2.0
        
        self.timestamps.append(
            (img,map)
        )
        
        self.save_timestamp()
    
    def save_timestamp(self):
        if not os.path.exists(self.output_path):
            os.mkdir(self.output_path)
        
        i = len(self.timestamps)
        timestamp = self.timestamps[-1]
        timestamp_dir = f"{self.output_path}/step_{i}"
        
        if not os.path.exists(timestamp_dir):
            os.mkdir(timestamp_dir)
        
        np.save(f"{timestamp_dir}/img.np",timestamp[0])
        np.save(f"{timestamp_dir}/map.np",timestamp[1])
    

def main(args=None):
    rclpy.init(args=args)
    node = SceneMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
