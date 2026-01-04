#!/usr/bin/env python3

from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage,Image

from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry,OccupancyGrid


import torch

import cv2
from cv_bridge import CvBridge

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

        # Publisher
        self.publisher_ = self.create_publisher(
            Twist,
            '/KapiBara/motors/cmd_vel_unstamped',
            10
        )
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/KapiBara/camera/points',
            self.points_callback,
            10
        )
        
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

        # Timer period (0.01 seconds = 100 Hz)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.obstacle_detected = False
        
        self.tick_counter = 0

        self.get_logger().info(f'CmdVel publisher started at {int(1 / timer_period)} Hz')
        
        self.x = 0.0
        self.y = 0.0
        
        self.yaw = 0.0
        
        self.target_yaw = np.pi/2.0
        
        self.yaw_pid_p = 40.0
        self.yaw_pid_d = 1.75
        self.last_yaw_error = 0.0
        
        # map coordinates
        self.map_x = 0
        self.map_y = 0
        self.resolution = 0.05
        
        self.map_step = 0
        
        self.yaw_when_collision = 0.0
        
        self.rotate_againt_obstacle = False
        self.wait_for_odom = True
        
        self.map = None
        self.image = None
        
        self.checkpoints:list[Checkpoint] = []
        
        self.bridge = CvBridge()
        
        self.last_odom_timestamp = None
        
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
        
        if self.last_odom_timestamp is None:
            self.last_odom_timestamp = float(msg.header.stamp.sec) + msg.header.stamp.nanosec * 1e-9
            return
        
        timestamp = float(msg.header.stamp.sec) + msg.header.stamp.nanosec * 1e-9
        
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation) + np.pi
        
        self.get_logger().info(f'Robot position: {self.x} {self.y} {self.yaw}')
        self.wait_for_odom = False
        
        msg = Twist()
        
        yaw_error = self.target_yaw - self.yaw
        
        dt = timestamp - self.last_odom_timestamp
        self.last_odom_timestamp = timestamp
        
        p = self.yaw_pid_p * yaw_error
        d = self.yaw_pid_d * ((yaw_error - self.last_yaw_error)/dt)
        msg.angular.z = p + d
        self.last_yaw_error = yaw_error
                
        msg.angular.z = np.clip(msg.angular.z, -3.0, 3.0)
        
        self.get_logger().info(f'PID: P: {p}, D: {d}, Cmd angular.z: {msg.angular.z}')
        
        msg.linear.x = 0.0    

        self.publisher_.publish(msg)
        
    def map_callaback(self, msg: OccupancyGrid):
        self.get_logger().info('Got map!')
        self.map = np.array(msg.data,dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map = self.map.astype(np.float32) / 100.0
        
        self.map_x = msg.info.origin.position.x
        self.map_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        
    def image_callback(self,msg:Image):
        
        self.get_logger().info('Got image with format: %s' % msg.encoding)
        
        self.image = self.bridge.imgmsg_to_cv2(msg)
        
        
    def points_callback(self, msg: PointCloud2):
        # Read points from PointCloud2
        points = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
            reshape_organized_cloud=True
        )
        
        sorted_points = np.sort(points)
        
        min_points = np.mean(sorted_points[0:10])
        
        self.obstacle_detected = min_points < 0.0
        
        if self.obstacle_detected:    
            self.get_logger().info(f'Obstacle detected!')
            
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
    
    def tick(self):
        
        if self.wait_for_odom:
            return
        
        self.tick_counter += 1
        
        if self.tick_counter % 25 == 0:
            self.tick_counter = 0
            
            if self.image is not None:
                self.get_logger().info('Adding checkpoint!')
                self.add_checkpoint()
                
                self.stop()
                return

        
        

    def timer_callback(self):
        self.timer.cancel()
        
        self.tick()
        
        self.timer.reset()


def main(args=None):
    rclpy.init(args=args)
    node = SceneMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
