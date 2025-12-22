#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry

import numpy as np


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
        
        self.odom_callback = self.create_subscription(
            Odometry,
            '/KapiBara/odom',
            self.odom_callback,
            10
        )

        # Timer period (0.01 seconds = 100 Hz)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.obstacle_detected = False

        self.get_logger().info('CmdVel publisher started at 100 Hz')
        
        self.x = 0.0
        self.y = 0.0
        
        self.yaw = 0.0
        
        self.yaw_when_collision = 0.0
        
        self.rotate_againt_obstacle = False
    
    def quaternion_to_yaw(self,q):
        """
        Convert a ROS2 geometry_msgs.msg.Quaternion to yaw (rad).
        """
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw
    
    def odom_callback(self,msg: Odometry):
        
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        self.get_logger().info(f'Robot position: {self.x} {self.y} {self.yaw}')
        
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
            if not self.rotate_againt_obstacle:
                self.yaw_when_collision = self.yaw
                self.rotate_againt_obstacle = True
            self.get_logger().info(f'Obstacle detected!')
        

    def timer_callback(self):
        msg = Twist()
        
        if abs(self.yaw - self.yaw_when_collision) > np.pi/4:
                self.rotate_againt_obstacle = False
        
        if not self.rotate_againt_obstacle:
            msg.linear.x = 0.25
            msg.angular.z = 0.0
        else:
                
            msg.linear.x = 0.0
            msg.angular.z = 3.0

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SceneMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
