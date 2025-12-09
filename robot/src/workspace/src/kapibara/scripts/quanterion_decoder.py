#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu,MagneticField
from kapibara_interfaces.msg import PiezoSense

import signal

import math

import numpy as np

class QuaternionToEulerNode(Node):
    def __init__(self):
        self.samples = 0
        
        self._gyro = np.zeros(3,dtype=np.float32)
        self._accel = np.zeros(3,dtype=np.float32)
        
        super().__init__('quaternion_to_euler_node')
        self.subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.subscriber = self.create_subscription(MagneticField, '/mag', self.mag_callback, 10)
        
        self.subscriber = self.create_subscription(PiezoSense, '/sense', self.sense_callback, 10)
        
    def sense_callback(self,msg):
        if msg.id == 3:
            self.get_logger().info('Sense: F: {} P: {}\n'.format(msg.frequency,msg.power))
        
    def mag_callback(self,msg):
        
        x = msg.magnetic_field.x
        y = msg.magnetic_field.y
        z = msg.magnetic_field.z
        
        # self.get_logger().info('Mag: X: {} Y: {} Z: {}\n'.format(x,y,z))
        
        # yaw = math.atan2(y,x)*(180.0/math.pi)
        
        # self.get_logger().info('Mag Pitch: {}\n'.format(yaw))
        
        

    def imu_callback(self, msg):
        # Extract quaternion from IMU message
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        x = msg.linear_acceleration.x
        y = msg.linear_acceleration.y
        z = msg.linear_acceleration.z
        
        accel = np.array([x,y,z],dtype=np.float32)
        
        x = msg.angular_velocity.x
        y = msg.angular_velocity.y
        z = msg.angular_velocity.z
        
        gyro = np.array([x,y,z],dtype=np.float32)
        
        self._accel += accel
        self._gyro += gyro
        
        if self.samples == 1000:
            self._gyro /= self.samples
            self._accel /= self.samples
            
            self.get_logger().info('Accel: {}\n'.format(self._accel))
            self.get_logger().info('Gyro: {}\n'.format(self._gyro))
            
            self.samples = 0
            self._gyro = np.zeros(3,dtype=np.float32)
            self._accel = np.zeros(3,dtype=np.float32)
        
        
        self.samples += 1
        
        # gravity quanterion:
        # if z >= 0:
        #     aqx = math.sqrt( ( z+1 ) / 2 )
        #     aqy = -y / math.sqrt( 2*z +2 )
        #     aqz = x / math.sqrt( 2*z + 2 )
        #     aqw = 0.0
        # else:
        #     aqx = -y / math.sqrt( 2 - 2*z )
        #     aqy = math.sqrt( ( 1 - z ) / 2 )
        #     aqz = 0.0
        #     aqw = x / math.sqrt( 2 - 2*z )
        
        m23 = 2*(qx*qy - qw*qz)
        m33 = qw*qw - qz*qz - qy*qy + qx*qx
        
        m13 = 2*(qx*qz + qw*qy)
        
        m12 = 2*(qz*qy - qw*qx)
        m11 = qw*qw + qz*qz - qy*qy - qx*qx
        
        m22 = qw*qw - qz*qz + qy*qy - qx*qx
        m32 = 2*(qy*qx + qw*qz)

        # # Convert quaternion to Euler angles (yaw, pitch, roll)
        # yaw = math.atan2(qx*qy + qz*qw, 0.5 - ( qy * qy + qz * qz))
        # pitch = 2*math.atan2(1+2*(qw*qy - qx*qz),1-2*(qw*qy-qx*qz)) - math.pi/2
        # # # pitch = math.asin((qx*qz - qy*qw))
        # roll = math.atan2(qw * qx + qy * qz, 0.5 - ( qx * qx + qy * qy))     
        
        # y - z - x
        yaw = math.atan2(m32,m22)
        pitch = math.atan2(m13,m11)
        roll = math.atan2(-m12,math.sqrt(1 - m12*m12))
        
        # angle = 2*math.acos(qw)
                
        # yaw = ( qz/math.sqrt(1 - qw*qw) ) * angle
        # pitch = ( qy/math.sqrt(1 - qw*qw) ) * angle
        # roll = ( qx/math.sqrt(1 - qw*qw) ) * angle

        yaw = yaw * ( 180.0/math.pi )
        pitch =  pitch * ( 180.0/math.pi )
        roll = roll * ( 180.0/math.pi )

        # self.get_logger().info('Yaw: {} Pitch: {} Roll: {}\n'.format(yaw,pitch,roll))
        # self.get_logger().info('X: {} Y: {} Z: {} W: {}\n'.format(aqx,aqy,aqz,aqw))
        # self.get_logger().info('10 X: {} Y: {} Z: {} W: {}\n'.format(qx,qy,qz,qw))
        # # self.get_logger().info('A1 X: {} Y: {} Z: {}\n'.format(x,y,z))
        
        # yaw = math.atan2(y,x)*( 180.0/math.pi )
        # pitch = math.atan2(x,z)*( 180.0/math.pi )
        # roll = math.atan2(y,z)*( 180.0/math.pi )
        
        
        # self.get_logger().info('10: Yaw: {} Pitch: {} Roll: {}'.format(yaw,pitch,roll))
        
    def on_shutdown(self):
        with open("./test.txt","w") as file:
            file.write("Hello World!")
        
        rclpy.shutdown()
        
def dupa(sig, frame):
    with open("./test.txt","w") as file:
        file.write("Hello World!")
    
    rclpy.shutdown()
        

def main(args=None):
    rclpy.init(args=args)
    node = QuaternionToEulerNode()
    
    signal.signal(signal.SIGINT,dupa)
    
    rclpy.spin(node)
        
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()