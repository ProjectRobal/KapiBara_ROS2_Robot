#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

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

import os

import cv2
from cv_bridge import CvBridge

from nav_msgs.msg import Odometry

from timeit import default_timer as timer

from copy import copy
import numpy as np

from ament_index_python.packages import get_package_share_directory,get_package_prefix

from emotion_estimer.kapibara_audio import KapibaraAudio
from emotion_estimer.midas import midasDepthEstimator

'''

We have sensors topics that will estimate angers 

'''


class EmotionEstimator(Node):

    def __init__(self):
        super().__init__('emotion_estimator')
        
        self.current_ranges=[]
        
        self.id_to_emotion_name = ["angry","fear","happiness","uncertainty","boredom"]
        
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed',descriptor=ParameterDescriptor(name='camera_topic', type=0, description="A topic to subscribe for compressed image"))
        
        self.declare_parameter('range_threshold', 0.1)
        self.declare_parameter('accel_threshold', 10.0)
        self.declare_parameter('angular_threshold', 1.0)
        
        # a topic to send ears position
        
        self.declare_parameter('ears_topic','/ears_controller/commands')
        
        # a list of topics of tof sensors
        self.declare_parameter('points_topic', '/midas/points' )
        
        # a orientation callback
        self.declare_parameter('imu', '/imu')
        
        # a topic to listen for audio from microphone
        self.declare_parameter('mic', '/mic')
        
        # a topic to listen for odometry 
        self.declare_parameter('odom','/motors/odom')
        
        # piezo sense sensors
        self.declare_parameter('sense_topic', '/sense')
                
        # anguler values for each emotions state
        self.emotions_angle=[0.0,180.0,45.0,135.0,90.0]    
        
        self.ears_publisher = self.create_publisher(Float64MultiArray, self.get_parameter('ears_topic').get_parameter_value().string_value, 10)
       
        self.emotion_publisher = self.create_publisher(Emotions,"/emotions",10)
        
        model_path = os.path.join(get_package_share_directory('emotion_estimer'),'model','model_edgetpu.tflite')
        
        self.get_logger().info("Loading KapiBara Audio model from: "+model_path)
        self.hearing = KapibaraAudio(path= model_path ,tflite=True)
        self.get_logger().info("Loaded KapiBara Audio model")
        
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
        
        
        self.accel_threshold = self.get_parameter('accel_threshold').get_parameter_value().double_value
        self.last_accel = 0.0
        self.thrust = 0.0
        self.last_uncertanity = 0.0
        
        self.angular_threshold = self.get_parameter('angular_threshold').get_parameter_value().double_value
        self.last_angular = 0.0
        self.angular_jerk = 0.0
        self.last_angular_fear = 0.0
        
        self.procratination_counter = 0
        # self.procratination_timer = self.create_timer(1.0, self.procratination_timer_callback)
        
        # parameter that describe fear distance threshold for laser sensor
        
        self.range_threshold = self.get_parameter('range_threshold').get_parameter_value().double_value
        
        # Orienataion callback
        
        imu_topic = self.get_parameter('imu').get_parameter_value().string_value
        
        self.get_logger().info("Creating subscription for IMU sensor at topic: "+imu_topic)
        self.imu_subscripe = self.create_subscription(Imu,imu_topic,self.imu_callback,10)
        
        # Sense callback
        
        sense_topic = self.get_parameter('sense_topic').get_parameter_value().string_value
        
        self.get_logger().info("Creating subscription for Pizeo Sense at topic: "+sense_topic)
        self.sense_subscripe = self.create_subscription(PiezoSense,sense_topic,self.sense_callabck,10)
    
        
        # Microphone callback use audio model 
        
        mic_topic = self.get_parameter('mic').get_parameter_value().string_value
        
        self.mic_buffor = np.zeros(2*16000,dtype=np.float32)
        
        self.get_logger().info("Creating subscription for Microphone at topic: "+mic_topic)
        self.mic_subscripe = self.create_subscription(Microphone,mic_topic,self.mic_callback,10)
        
        # Subcription for odometry
        
        odom_topic = self.get_parameter('odom').get_parameter_value().string_value
        self.get_logger().info("Creating subscription for odometry sensor at topic: "+odom_topic)
        self.odom_subscripe = self.create_subscription(Odometry,odom_topic,self.odom_callback,10)
        
        # Point Cloud 2 callbck
        
        self.move_lock = False
        
        self.pain_value = 0
        
        self.good_sense = 0
        
        self.uncertain_sense = 1.0
        
        self.points_covarage = 0
        
        points_topic = self.get_parameter('points_topic').get_parameter_value().string_value
        
        self.get_logger().info("Creating subscription for Point Cloud sensor at topic: "+points_topic)
        self.point_subscripe = self.create_subscription(PointCloud2,points_topic,self.points_callback,10)
                
        self.ears_timer = self.create_timer(0.05, self.ears_subscriber_timer)
    
    # subscribe ears position based on current emotion 
    def ears_subscriber_timer(self):
        emotions = self.calculate_emotion_status()
        
        _emotions = Emotions()
        
        _emotions.angry = emotions[0]
        _emotions.fear = emotions[1]
        _emotions.happiness = emotions[2]
        _emotions.uncertainty = emotions[3]
        _emotions.boredom = emotions[4]
        
        self.emotion_publisher.publish(_emotions)
        
        max_id = 4
        
        if np.sum(emotions) >= 0.01:
            max_id:int = np.argmax(emotions)
            
        self.get_logger().debug("Current emotion: "+str(self.id_to_emotion_name[max_id]))
            
        self.get_logger().debug("Sending angle to ears: "+str(self.emotions_angle[max_id]))
        
        angle:float = (self.emotions_angle[max_id]/180.0)*np.pi
        
        array=Float64MultiArray()
        
        array.data=[np.pi - angle, angle]
        
        self.ears_publisher.publish(array)
    
    
    def calculate_emotion_status(self) -> np.ndarray:
        emotions=np.zeros(5)
        
        # anger
        # fear
        # happiness
        # uncertainty
        # bordorm
        emotions[0] = 0.0
        emotions[1] = ( self.last_angular_fear > 0.1 )*self.last_angular_fear + ( self.points_covarage > 120 )*0.5 + 0.35*self.pain_value
        emotions[2] = self.good_sense
        emotions[3] = (self.last_uncertanity > 0.1)*self.last_uncertanity + self.uncertain_sense*0.5
        emotions[4] = np.floor(self.procratination_counter/5.0)
        
        self.pain_value = self.pain_value / 1.25
        self.good_sense = self.good_sense / 1.5
        self.uncertain_sense = self.uncertain_sense / 1.35
        
        if self.pain_value <= 0.1:
            self.pain_value = 0.0
            
        if self.good_sense <= 0.1:
            self.good_sense = 0.0
            
        if self.uncertain_sense <= 0.1:
            self.uncertain_sense = 0.0
        
        return emotions
        
           
    def procratination_timer_callback(self):
        if self.procratination_counter < 10000:
            self.procratination_counter = self.procratination_counter + 1
            
    def camera_listener(self, msg:CompressedImage):
        self.get_logger().debug('I got image with format: %s' % msg.format)
        
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        
        start = timer()
        
        colorDepth = self.midas.estimateDepth(image)
        
        self.get_logger().debug('Estimation time: %f s' % ( timer() - start ))
        
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
            
    def sense_callabck(self,sense:PiezoSense):
        
        if self.move_lock:
            return
        
        if sense.id == 3:
            self.get_logger().debug('Sense: F: {} P: {}\n'.format(sense.frequency,sense.power))
            
            if sense.power > 135:
                self.pain_value = 1.0
                
            if sense.frequency <= 4 and sense.power <= 50 and sense.power > 10:
                self.good_sense = 1.0
                
            if sense.frequency > 4:
                self.uncertain_sense = 1.0
            
            
    def points_callback(self,points:PointCloud2):
        
        if len(points.fields) < 4:
            self.get_logger().error("Invalid point cloud message")
            return
        
        width = points.width
        height = points.height
        
        points = np.frombuffer(points.data,dtype=np.float32)
        
        points = points[2:len(points):4]
        
        # 4 becaue we have 4 fields
        if len(points) < width*height:
            self.get_logger().error("Invalid number of point in cloud message")
            return
        
        points = points.reshape((height,width,1))
        
        self.points_covarage = ( points < 1.2 ).sum() 
                
        self.get_logger().debug("Depth covarage: "+str(self.points_covarage))
        
        
        
    
    def odom_callback(self,odom:Odometry):
        
        velocity = odom.twist.twist.linear.x*odom.twist.twist.linear.x + odom.twist.twist.linear.y*odom.twist.twist.linear.y + odom.twist.twist.linear.z*odom.twist.twist.linear.z
        angular = odom.twist.twist.angular.x*odom.twist.twist.angular.x + odom.twist.twist.angular.y*odom.twist.twist.angular.y + odom.twist.twist.angular.z*odom.twist.twist.angular.z
        
        self.get_logger().debug("Odom recive velocity of "+str(velocity)+" and angular velocity of "+str(angular))
        if velocity > 0.0001 or angular > 0.01:
            self.move_lock = True
            self.procratination_counter = 0
        else:
            self.move_lock = False        
        
    def imu_callback(self,imu:Imu):
        accel=imu.linear_acceleration
        
        accel_value = np.sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z)/3  
        
        self.thrust = abs(accel_value-self.last_accel)
        
        self.last_accel = accel_value
        
        self.last_uncertanity = (self.thrust/self.accel_threshold) + 0.5*self.last_uncertanity
                
        self.get_logger().debug("Thrust uncertanity value "+str(self.last_uncertanity))
        
        angular = imu.angular_velocity
        
        angular_value = np.sqrt(angular.x*angular.x + angular.y*angular.y + angular.z*angular.z)/3
        
        self.angular_jerk = abs(angular_value-self.last_angular)
        
        self.last_angular = angular_value
        
        self.last_angular_fear = (self.angular_jerk/self.angular_threshold) + 0.4*self.last_angular_fear
        
        self.get_logger().debug("Jerk value fear "+str(self.last_angular_fear))
    
    def mic_callback(self,mic:Microphone):
        # I have to think about it
        
        self.mic_buffor = np.roll(self.mic_buffor,mic.buffor_size)
        
        left = np.array(mic.channel1,dtype=np.float32)/np.iinfo(np.int32).max
        right = np.array(mic.channel2,dtype=np.float32)/np.iinfo(np.int32).max
        
        combine = ( left + right ) / 2.0
        
        self.mic_buffor[:mic.buffor_size] = combine[:]
        
        start = timer()
        
        output = self.hearing.input(self.mic_buffor)
                
        self.get_logger().debug("Hearing time: "+str(timer() - start)+" s")
        
        self.get_logger().debug("Hearing output: "+str(output))



def main(args=None):
    rclpy.init(args=args)

    emotion_estimator = EmotionEstimator()

    rclpy.spin(emotion_estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    emotion_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()