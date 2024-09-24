#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from sensor_msgs.msg import Range,Imu
from geometry_msgs.msg import Quaternion

from kapibara_interfaces.srv import Emotions
from kapibara_interfaces.msg import Microphone

from nav_msgs.msg import Odometry

from copy import copy
import numpy as np

'''

We have sensors topics that will estimate angers 

'''


class EmotionEstimator(Node):

    def __init__(self):
        super().__init__('emotion_estimator')
        
        self.current_ranges=[]
        
        self.declare_parameter('range_threshold', 0.1)
        self.declare_parameter('accel_threshold', 10.0)
        self.declare_parameter('angular_threshold', 1.0)
        
        # a topic to send ears position
        
        self.declare_parameter('ears_topic','/ears_controller/commands')
        
        # a list of topics of tof sensors
        self.declare_parameter('tofs', ['/laser_front'])
        
        # a orientation callback
        self.declare_parameter('imu', '/Gazebo/orientation')
        
        # a topic to listen for audio from microphone
        self.declare_parameter('mic', '')
        
        # a topic to listen for odometry 
        self.declare_parameter('odom','/motors/odom')
        
        self.service = self.create_service(Emotions,'emotions',self.emotions_callback)    
        
        # anguler values for each emotions state
        self.emotions_angle=[180.0,0.0,135.0,45.0,90.0]    
        
        self.ears_publisher=self.create_publisher(Float64MultiArray, self.get_parameter('ears_topic').get_parameter_value().string_value, 10)
        
        
        self.accel_threshold = self.get_parameter('accel_threshold').get_parameter_value().double_value
        self.last_accel = 0.0
        self.thrust = 0.0
        self.last_uncertanity = 0.0
        
        self.angular_threshold = self.get_parameter('angular_threshold').get_parameter_value().double_value
        self.last_angular = 0.0
        self.angular_jerk = 0.0
        self.last_angular_fear = 0.0
        
        self.procratination_counter = 0
        self.procratination_timer = self.create_timer(0.5, self.procratination_timer_callback)
        
        # parameter that describe fear distance threshold for laser sensor
        
        self.range_threshold = self.get_parameter('range_threshold').get_parameter_value().double_value
        self.laser_subscriptions=[]
        
        # create N of those subscriptions
        
        tofs = self.get_parameter('tofs').get_parameter_value().string_array_value
        
        for id,tof in enumerate(tofs):
            self.get_logger().info("Creating subscription for TOF sensor at topic: "+tof+" with id "+str(id))
            self.laser_subscriptions.append(self.create_subscription(Range,tof,lambda msg,x=id: self.laser_callback(msg,x),10))
            self.current_ranges.append(0.0)
        
        # Orienataion callback
        
        imu_topic = self.get_parameter('imu').get_parameter_value().string_value
        
        self.get_logger().info("Creating subscription for IMU sensor at topic: "+imu_topic)
        self.imu_subscripe = self.create_subscription(Imu,imu_topic,self.imu_callback,10)
        
        # Microphone callback use audio model 
        
        mic_topic = self.get_parameter('mic').get_parameter_value().string_value
        
        if len(mic_topic)>0:
            self.get_logger().info("Creating subscription for IMU sensor at topic: "+mic_topic)
            self.mic_subscripe = self.create_subscription(Microphone,mic_topic,self.mic_callback,10)
        
        # Subcription for odometry
        
        odom_topic = self.get_parameter('odom').get_parameter_value().string_value
        self.get_logger().info("Creating subscription for odometry sensor at topic: "+odom_topic)
        self.odom_subscripe = self.create_subscription(Odometry,odom_topic,self.odom_callback,10)
        
        self.ears_timer = self.create_timer(1.0, self.ears_subscriber_timer)
    
    # subscribe ears position based on current emotion 
    def ears_subscriber_timer(self):
        emotions = self.calculate_emotion_status()
        
        if np.sum(emotions) < 0.01:
            max_id=4
        else:
            max_id:int = np.argmax(emotions)
            
        self.get_logger().debug("Sending angle to ears: "+str(self.emotions_angle[max_id]))
        
        angle:float = (self.emotions_angle[max_id]/180.0)*np.pi
        
        array=Float64MultiArray()
        
        array.data=[angle,angle]
        
        self.ears_publisher.publish(array)
    
    def calculate_emotion_status(self) -> np.ndarray:
        emotions=np.zeros(5)
        
        emotions[0] = 0.0
        emotions[1] = ( 1.0 - min( ( min( self.current_ranges )/self.range_threshold ),1.0) ) + ( self.last_angular_fear > 0.1 )*self.last_angular_fear
        emotions[2] =0.0
        emotions[3] = (self.last_uncertanity > 0.1)*self.last_uncertanity
        emotions[4] = np.floor(self.procratination_counter/5.0)
        
        return emotions
        
           
    def procratination_timer_callback(self):
        if self.procratination_counter < 10000:
            self.procratination_counter = self.procratination_counter + 1
    
    def odom_callback(self,odom:Odometry):
        
        velocity = odom.twist.twist.linear.x*odom.twist.twist.linear.x + odom.twist.twist.linear.y*odom.twist.twist.linear.y + odom.twist.twist.linear.z*odom.twist.twist.linear.z
        angular = odom.twist.twist.angular.x*odom.twist.twist.angular.x + odom.twist.twist.angular.y*odom.twist.twist.angular.y + odom.twist.twist.angular.z*odom.twist.twist.angular.z
        
        self.get_logger().debug("Odom recive velocity of "+str(velocity)+" and angular velocity of "+str(angular))
        if velocity > 0.0001 or angular > 0.01:
            self.procratination_counter = 0
        
    
    def laser_callback(self,laser:Range,id):
        self.get_logger().debug("Got range with id "+str(id)+" with range: "+str(laser.range))
        self.current_ranges[id]=laser.range
        
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
        pass
    
    def emotions_callback(self, request, response):
        
        self.get_logger().debug("Current ranges kept by program: "+str(self.current_ranges))
        
        emotions = self.calculate_emotion_status()
        
        response.angry=emotions[0]
        response.fear=emotions[1]
        response.happiness=emotions[2]
        response.uncertainty=emotions[3]
        response.boredom=emotions[4]
                
        return response


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