#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import os

class RgbdFolderPublisher(Node):
    def __init__(self):
        super().__init__('rgbd_folder_publisher')
        
        # Parameters
        self.declare_parameter('color_path', './images/color')
        self.declare_parameter('depth_path', './images/depth')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('frame_id', 'camera_depth_optical_frame')
        
        color_dir = self.get_parameter('color_path').get_parameter_value().string_value
        depth_dir = self.get_parameter('depth_path').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Publishers
        self.color_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_rect_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/color/camera_info', 10)
        
        self.bridge = CvBridge()
        
        # Syncing files: assumes identical filenames in both folders
        self.color_files = sorted([os.path.join(color_dir, f) for f in os.listdir(color_dir) if f.endswith('.png')])
        self.depth_files = sorted([os.path.join(depth_dir, f) for f in os.listdir(depth_dir) if f.endswith('.png')])

        if len(self.color_files) != len(self.depth_files):
            self.get_logger().warn(f"Mismatch: {len(self.color_files)} color vs {len(self.depth_files)} depth images.")

        # Cache Camera Info
        sample_img = cv2.imread(self.color_files[0])
        self.height, self.width = sample_img.shape[:2]
        self.camera_info = self.get_static_camera_info()

        self.index = 0
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def get_static_camera_info(self):
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width, info.height = self.width, self.height
        info.distortion_model = "plumb_bob"
        f = float(self.width)
        info.k = [f, 0.0, self.width/2.0, 0.0, f, self.height/2.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, self.width/2.0, 0.0, 0.0, f, self.height/2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def timer_callback(self):
        if self.index >= len(self.color_files):
            self.index = 0

        # Read images
        color_img = cv2.imread(self.color_files[self.index])
        # Read depth: IMREAD_UNCHANGED is vital for 16-bit depth PNGs!
        depth_img = cv2.imread(self.depth_files[self.index], cv2.IMREAD_UNCHANGED)

        if color_img is not None and depth_img is not None:
            now = self.get_clock().now().to_msg()

            # 1. Color Message (BGR8)
            color_msg = self.bridge.cv_to_imgmsg(color_img, encoding="bgr8")
            color_msg.header.stamp = now
            color_msg.header.frame_id = self.frame_id

            # 2. Depth Message (16UC1 or 32FC1)
            # If your PNG is 16-bit, use '16UC1'. If it's 8-bit, use 'mono8'
            depth_encoding = "16UC1" if depth_img.dtype == 'uint16' else "mono8"
            depth_msg = self.bridge.cv_to_imgmsg(depth_img, encoding=depth_encoding)
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = self.frame_id

            # 3. Info Message
            self.camera_info.header.stamp = now

            # Publish all
            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)
            self.info_pub.publish(self.camera_info)
            
            self.index += 1

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RgbdFolderPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()