#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from sensor_msgs.msg import Range,Imu,PointCloud2
from geometry_msgs.msg import Quaternion

from kapibara_interfaces.msg import Emotions
from kapibara_interfaces.msg import Microphone
from kapibara_interfaces.msg import PiezoSense

from sensor_msgs.msg import CompressedImage,Image
from sensor_msgs.msg import PointCloud2,PointField

from sensor_msgs.msg import CompressedImage,Image
from sensor_msgs.msg import PointCloud2,PointField

from rcl_interfaces.msg import ParameterDescriptor

from kapibara_interfaces.msg import FaceEmbed

from kapibara_interfaces.srv import StopMind

import cv2
from cv_bridge import CvBridge

import numpy as np

from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance,OrderBy,PayloadSchemaType
from qdrant_client.http import models

EMBEDDING_SHAPE = (32*3,32)
ACTION_SET_NAME = "actions_sets"

def generate_embedding(img):
    
    img_in = cv2.resize(img,(224,224),interpolation=cv2.INTER_AREA)
    
    img_in = cv2.cvtColor(img_in, cv2.COLOR_BGR2GRAY)
    
    edges = cv2.Canny(img_in, 200, 300)
        
    ksize = 3              # Kernel size (odd number)
    sigma = 4.0             # Standard deviation of Gaussian
    lambd = 10.0            # Wavelength
    gamma = 1.0            # Aspect ratio
    
    gabor_img = np.zeros((224,224),dtype=np.float32)
        
    for theta in np.arange(0, np.pi, np.pi / 4):
        kernel = cv2.getGaborKernel(
            (ksize, ksize),
            sigma=sigma,
            theta=theta,
            lambd=lambd,
            gamma=gamma,
            psi=0
        )
        gabor_img += cv2.filter2D(img_in, cv2.CV_32F, kernel)
        
    gabor_img = cv2.normalize(gabor_img, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)
    
    img_in_sm = cv2.resize(img_in,(32,32),interpolation=cv2.INTER_AREA)
    
    edges_sm = cv2.resize(edges,(32,32),interpolation=cv2.INTER_AREA)
    
    gabor_img_sm = cv2.resize(gabor_img,(32,32),interpolation=cv2.INTER_AREA)
    
    embedding_img = cv2.hconcat([img_in_sm,edges_sm,gabor_img_sm])
    
    return embedding_img


class MapMind(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('tick_time', 0.05)
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        timer_period = self.get_parameter('tick_time').get_parameter_value().double_value  # seconds
        self.timer = self.create_timer(timer_period, self.tick_callback)
        
        self.image_sub = self.create_subscription(
            CompressedImage,
            'camera',
            self.image_callback,
            10)
        
        self.img_embedding = np.zeros(EMBEDDING_SHAPE,dtype=np.uint8)
        
        self.bridge = CvBridge()
        
        self.db = QdrantClient(host='0.0.0.0',port=6333)
        
        self.initialize_db()
        
        self.emotion_state = 0.0
        
        self.last_embedding = np.zeros(EMBEDDING_SHAPE,dtype=np.uint8)
        
        self.action_list = []
        self.action_executing = False
        self.action_iter = iter(self.action_list)
        
    def initialize_db(self):
        
        if not self.db.collection_exists(ACTION_SET_NAME):
            self.get_logger().warning("Action set database not exist, creating new one!")
            self.db.create_collection(
                collection_name=ACTION_SET_NAME,
                vectors_config=VectorParams(size=EMBEDDING_SHAPE[0]*EMBEDDING_SHAPE[1], distance=Distance.EUCLID),
            )
    
    def send_command(self,linear:float,angular:float):
        
        twist = Twist()
        
        linear = min(linear,1.0)
        linear = max(linear,-1.0)
        
        angular = min(angular,1.0)
        angular = max(angular,-1.0)
        
        twist.linear.x = linear*self.max_linear_speed
        twist.angular.z = angular*self.max_angular_speed
        
        self.twist_pub.publish(twist)
        
    def stop(self):
        self.send_command(0.0,0.0)
        
    def image_callback(self,msg:CompressedImage):
        
        self.get_logger().debug('I got image with format: %s' % msg.format)
        
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        
        self.img_embedding = generate_embedding(image)
        
    def emotion_callabck(self,msg:Emotions):
        self.get_logger().debug("Got emotions")
        
        self.emotion_state = msg.happiness*320.0 + msg.fear*-120.0 + msg.uncertainty*-40.0 + msg.angry*-60.0 + msg.boredom*-20.0
        
    def tick_callback(self):
        self.get_logger().info('Mind tick')
        
        if self.action_executing:
            action = next(self.action_iter,None)
            
            if self.emotion_state < -0.25:
                # perform crossover and mutations
                self.action_executing = False
                self.stop()
                return
            
            if action is None or self.emotion_state > 0.0:
                # perform action set trim
                self.action_executing = False
                self.stop()
                return
            
            self.send_command(action[0],action[1])
            
            return
        
        if self.emotion_state >= 0.0:
            self.stop()
            return
        
        self.last_embedding = self.img_embedding
        
        hits = self.db.query_points(
            collection_name=ACTION_SET_NAME,
            query=self.last_embedding,
            limit=10
        )
        
        points = hits.points
        
        
        
        

def main(args=None):
    rclpy.init(args=args)

    map_mind = MapMind()

    rclpy.spin(map_mind)

    map_mind.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()