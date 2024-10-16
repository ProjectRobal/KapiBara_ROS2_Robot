#!/bin/python

import sys

import rclpy
from rclpy.node import Node

from modus_board_interfaces.srv import EnableBoards,ResetBoards

from vel_saltis_services.srv import SetImuCFG,SetPIDCFG,SetFusionCFG

from vel_saltis_services.msg import ImuCFG,PIDCFG,FusionCFG

from kapibara_interfaces.msg import PID

from geometry_msgs.msg import Vector3

import json



class StartSequence(Node):

    def __init__(self):
        super().__init__('start_sequence')
        
        self.declare_parameter('imu_cfg_path', '')
        self.declare_parameter('pid_cfg_path', '')
        self.declare_parameter('fusion_cfg_path', '')
        
        
        self.imu_cfg_path = self.get_parameter('imu_cfg_path').get_parameter_value().string_value
        self.pid_cfg_path = self.get_parameter('pid_cfg_path').get_parameter_value().string_value
        self.fusion_cfg_path = self.get_parameter('fusion_cfg_path').get_parameter_value().string_value
        
        self.failed = False
        
        self.reset = self.create_client(ResetBoards, 'reset_boards')
        
        if not self.reset.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reset Boards service not available, waiting again...')
            self.failed = True
            return
            
        self.enable = self.create_client(EnableBoards, 'enable_boards')
        
        if not self.enable.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Enable Boards service not available, waiting again...')
            self.failed = True
            return
        
        self.imu_cfg = self.create_client(SetImuCFG, 'set_imu_config')
        
        if not self.imu_cfg.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set IMU config service not available, waiting again...')
            self.failed = True
            return
        
        self.fusion_cfg = self.create_client(SetFusionCFG, 'set_fusion_config')
        
        if not self.fusion_cfg.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set Fusion config service not available, waiting again...')
            self.failed = True
            return
        
        self.pid_cfg = self.create_client(SetPIDCFG, 'set_fusion_config')
        
        if not self.pid_cfg.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set PID config service not available, waiting again...')
            self.failed = True
            return

    def reset_boards(self):
        req = ResetBoards.Request()
        self.future = self.reset.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def enable_boards(self,enabled:bool):
        req = EnableBoards.Request()
        
        req.enable = enabled
        
        self.future = self.enable.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    
    def set_imu_config(self):
        req = SetImuCFG.Request()
        
        try:
            
            cfg = json.load(self.imu_cfg_path)

            req.config.gyroscope_offset.x = cfg["gyroscope"]["x"]
            req.config.gyroscope_offset.y = cfg["gyroscope"]["y"]
            req.config.gyroscope_offset.z = cfg["gyroscope"]["z"]
            
            req.config.accelerometer_offset.x = cfg["accelerometer"]["x"]
            req.config.accelerometer_offset.y = cfg["accelerometer"]["y"]
            req.config.accelerometer_offset.z = cfg["accelerometer"]["z"]
            
        except json.JSONDecodeError:
            self.get_logger().error('Cannot load IMU config json')
            return False
        except KeyError:
            self.get_logger().error('There is some missing options in IMU config json')
            return False
        
        self.future = self.imu_cfg.call_async(req)
        rclpy.spin_until_future_complete(self, self.future,timeout_sec=20)
        return self.future.result().ok
    
    def set_pid_config(self):
        req = SetPIDCFG.Request()
        
        try:
            
            cfg = json.load(self.pid_cfg_path)

            req.config.left.p = cfg["left"]["p"]
            req.config.left.i = cfg["left"]["i"]
            req.config.left.d = cfg["left"]["d"]
            
            req.config.right.p = cfg["right"]["p"]
            req.config.right.i = cfg["right"]["i"]
            req.config.right.d = cfg["right"]["d"]
            
            req.config.open = int(cfg["open"])
            
        except json.JSONDecodeError:
            self.get_logger().error('Cannot load PID config json')
            return False
        except KeyError:
            self.get_logger().error('There is some missing options in PID config json')
            return False
        
        self.future = self.pid_cfg.call_async(req)
        rclpy.spin_until_future_complete(self, self.future,timeout_sec=20)
        return self.future.result().ok
    
    def set_fusion_config(self):
        req = SetFusionCFG.Request()
        
        try:
            
            cfg = json.load(self.fusion_cfg_path)

            req.config.beta = cfg["beta"]

        except json.JSONDecodeError:
            self.get_logger().error('Cannot load Fusion config json')
            return False
        except KeyError:
            self.get_logger().error('There is some missing options in Fusion config json')
            return False
        
        self.future = self.fusion_cfg.call_async(req)
        rclpy.spin_until_future_complete(self, self.future,timeout_sec=20)
        return self.future.result().ok


def main(args=None):
    rclpy.init(args=args)

    start = StartSequence()
    
    if start.failed:
        start.destroy_node()
        rclpy.shutdown()
        return
    
    start.get_logger().info('1.Disabling boards')
    response = start.enable_boards(False)
    
    start.get_logger().info('2.Reseting boards')
    start.reset_boards()
    
    try:
    
        start.get_logger().info('3.Setting IMU config')
        if not start.set_imu_config():
            start.get_logger().error('Cannot set IMU config')
    
        start.get_logger().info('4.Setting PID config')
        
        if not start.set_pid_config():
            start.get_logger().error('Cannot set PID config')
    
        start.get_logger().info('5.Setting Fusion config')
        
        if not start.set_fusion_config():
            start.get_logger().error('Cannot set Fusion config')
    
    except Exception as e:
        start.get_logger().error('Generl error: '+str(e))
    
    start.get_logger().info('6.Enabling boards')
    response = start.enable_boards(True)
    
    start.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()