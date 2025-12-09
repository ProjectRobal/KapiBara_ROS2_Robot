from kapibara.devices.Device import Device
from kapibara.devices.Gyroscope import Gyroscope
from kapibara.libs.mpu6050 import mpu6050
from kapibara.devices.DistanceSensor import DistanceSensor
from kapibara.devices.Servos import Servos
from kapibara.devices.Ears import Ears

import rclpy
from rclpy.node import Node

def initialize_devices(node:Node,calib:bool=False)->list[Device]:

    device_list:list[Device]=[]

    #gyro= Gyroscope(node,0.0025,"imu",0,2)
    
    #gyro.setAccelerationRange(mpu6050.ACCEL_RANGE_16G)
    #gyro.setGyroscopeRange(mpu6050.GYRO_RANGE_1000DEG)
    
    #gyro.setFiltereRange(mpu6050.FILTER_BW_42)
    
    #if calib:
    #    gyro.calibrate()

    servos= Servos(node,0,"/ears_controller/commands",1,3)

    #ears= Ears(node,0.01,"mics",5)
    servos.set_offset(0,0.0)
    servos.set_offset(1,-180.0)
    servos.set_channel(0,0.0)
    servos.set_channel(1,0.0)

    distance_front_1 = DistanceSensor(node,0.04,"laser_front1",2,4)
    distance_front_2 = DistanceSensor(node,0.04,"laser_front2",2,5)
    #distance_front_3 = DistanceSensor(node,0.04,"laser_front3",2,6)

    device_list.append(servos)
    #device_list.append(gyro)
    device_list.append(distance_front_2)
    device_list.append(distance_front_1)
    #device_list.append(distance_front_3)

    return device_list