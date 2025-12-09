from kapibara.devices.Device import Device
from kapibara.libs.mpu6050 import mpu6050
import kapibara.DeviceExceptions as errors

import os
import json
import numpy as np
import time
import kapibara.config as config

from sensor_msgs.msg import Imu



class Gyroscope(Device):
    def __init__(self,node,time,name,id,port):
        super().__init__(node,time,name,id,port)
        
        self._acceleration=np.zeros(3,dtype=np.float64)
        self._gyroscope=np.zeros(3,dtype=np.float64)
        
        self.gyro_off=np.zeros(3,dtype=np.float32)
        self.accel_off=np.zeros(3,dtype=np.float32)
        self.last_time=0.0
        self.sample_time=0.0
        self.init()
        
    def init(self):
        try:
            
            self.SelectPort()
            self._mpu=mpu6050(0x68)
            self._mpu.set_accel_range(mpu6050.ACCEL_RANGE_2G)
            self._mpu.set_gyro_range(mpu6050.GYRO_RANGE_500DEG)
            self.calibration_data()
        
        except Exception as e:
            print(str(e))
            raise errors.DeviceInitError(self.name(),"Cannot connect to i2c")
        
    def callback(self):
        try:
            self.SelectPort()
            
            _imu=Imu()

            accel=np.array(list(self._mpu.get_accel_data().values()),dtype=np.float32)-self.accel_off
            gyro=np.array(list(self._mpu.get_gyro_data().values()),dtype=np.float32)-self.gyro_off
            
            _imu.angular_velocity.x=float(gyro[0])
            _imu.angular_velocity.y=float(gyro[1])
            _imu.angular_velocity.z=float(gyro[2])
            
            _imu.linear_acceleration.x=float(accel[0])
            _imu.linear_acceleration.y=float(accel[1])
            _imu.linear_acceleration.z=float(accel[2])

            self._publisher.publish(_imu)
            
        except:
            raise errors.DeviceGeneralFault(self.name(),"Cannot read gyroscope data")
    
    def get_angels(self,gyroscope,acceleration):
        '''
            Get data from IMU to calculate pitch,roll and yaw
        '''
        angels=np.zeros(3,dtype=np.float32)

        index=np.argmax(acceleration)

        if index == 0:
            angels[2]=np.arctan2(acceleration[1],acceleration[0])
            angels[1]=np.arctan2(acceleration[0],acceleration[2])
        elif index == 1:
            angels[2]=np.arctan2(acceleration[1],acceleration[0])
            angels[1]=np.arctan2(acceleration[1],acceleration[2])
        else:
            angels[0]=np.arctan2(acceleration[1],acceleration[2])
            angels[1]=np.arctan2(acceleration[0],acceleration[1])

        return (angels*(180.0/np.pi))-np.array([0,90.0,1.0],np.float32)

    def calibration_data(self):
        '''
            Read calibration data from environment        
        '''
        try:
            with open(config.IMU_CALIB_FILE,"r") as f:
                calib:dict=json.load(f)

                if "calibration" in calib.keys():
                    self.gyro_off=calib["calibration"][0]
                    self.accel_off=calib["calibration"][1]
        except:
            print("Cannot open config file: "+config.IMU_CALIB_FILE)

    def calibrate(self,N,accel_def=config.DEFAULT_ACCELERAION):
        '''
            Return calibration data that will stored
            N - number of samples
            calib[0] - data from gyroscope
            calib[1] - data from accelerometer
        '''
        self.SelectPort()
        calib=np.zeros((2,3),dtype=np.float64)

        for n in range(N):
            calib[0]=calib[0]+np.array(list(self._mpu.get_gyro_data().values()),dtype=np.float32)
            calib[1]=calib[1]+np.array(list(self._mpu.get_accel_data().values()),dtype=np.float32)
            # about 250 Hz
            time.sleep(0.004)

        calib=calib/N

        calib[1]=calib[1]-accel_def

        # save calibration data

        with open(config.IMU_CALIB_FILE,"w") as f:
            json.dump({
                "calibration":calib.tolist()
            },f)

        return calib

    def start(self):
        self._publisher = self._node.create_publisher(Imu, self._name, 10)
        self._timer=self._node.create_timer(self._time, self.callback)
        
    def setAccelerationRange(self,_range):
        self.SelectPort()
        self._mpu.set_accel_range(_range)
        
    def setGyroscopeRange(self,_range):
        self.SelectPort()
        self._mpu.set_gyro_range(_range)
        
    def setFiltereRange(self,_range):
        self.SelectPort()
        self._mpu.set_filter_range(_range)
