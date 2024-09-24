#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from kapibara_interfaces.msg import Face,RectI,Vector2DI

from rcl_interfaces.msg import ParameterDescriptor

from deepface import DeepFace

import numpy as np
import base64

class FaceSpotter(Node):

    def __init__(self):
        super().__init__('face_spotter')
        
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed',descriptor=ParameterDescriptor(name='camera_topic', type=0, description="A topic to subscribe for compressed image"))
        
        self.declare_parameter('model_name', 'DeepID',descriptor=ParameterDescriptor(name="model_name", type=0, description="A model used for face analyze"))
        self.declare_parameter('detector_backend', 'yunet',descriptor=ParameterDescriptor(name="detector_backend", type=0, description="A backend used for face detection on image"))
        
        # initialize model
        self.get_logger().info('Initializing deepface...')
        
        self.model_name:str=self.get_parameter('model_name').get_parameter_value().string_value
        self.get_logger().info('Using model: %s' % self.model_name)
        
        self.detector_backend:str=self.get_parameter('detector_backend').get_parameter_value().string_value
        self.get_logger().info('Using backend: %s' % self.detector_backend)
        
        
        self.face_publisher = self.create_publisher(Face, '/spoted_faces', 10)
        
        self.subscription = self.create_subscription(
            CompressedImage,
            self.get_parameter('camera_topic').get_parameter_value().string_value,
            self.camera_listener,
            10)
        self.subscription  # prevent unused variable warning
        
    def check_for_faces(self,img):
        
        try:
        
            return DeepFace.represent(img_path = img,model_name=self.model_name,align=True,detector_backend=self.detector_backend,enforce_detection=True)
        
        except:
            return None

    def camera_listener(self, msg:CompressedImage):
        self.get_logger().debug('I got image with format: %s' % msg.format)
        
        image_buffer="data:image/png;base64,"+base64.b64encode(msg.data).decode("utf-8")
        
        embeddings_list=self.check_for_faces(image_buffer)
        
        if embeddings_list is None:
            return
        
        self.get_logger().debug('Found faces: %i' % len(embeddings_list))
        
        # format each output and send them to topic
        
        for embedding in embeddings_list:
            
            face=Face()
            
            face.embedding=embedding['embedding']
            facial_area=embedding['facial_area']
            face.facial_area=RectI(x=facial_area['x'],y=facial_area['y'],width=facial_area['w'],height=facial_area['h'])
            
            face.left_eye=Vector2DI(x=facial_area['left_eye'][0],y=facial_area['left_eye'][1])
            face.right_eye=Vector2DI(x=facial_area['right_eye'][0],y=facial_area['right_eye'][1])
            
            self.face_publisher.publish(face)

def main(args=None):
    rclpy.init(args=args)

    face_spotter = FaceSpotter()

    rclpy.spin(face_spotter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    face_spotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()