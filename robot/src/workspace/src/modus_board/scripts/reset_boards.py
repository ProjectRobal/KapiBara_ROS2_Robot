#!/usr/bin/env python3

'''
 A service used for reseting boards attached to Modus board.
'''

import gpiod
import time

import rclpy
from rclpy.node import Node

from modus_board_interfaces.srv import ResetBoards


class ResetBoardsService(Node):

    def __init__(self):
        super().__init__('enable_boards_service')
        self.declare_parameter("pin",3)
        
        pin = self.get_parameter("pin").get_parameter_value().integer_value
        
        self.chip = gpiod.Chip('gpiochip4')
        
        self.reset_pin = self.chip.get_line(pin)
        
        self.reset_pin.request(consumer="RST", type=gpiod.LINE_REQ_DIR_OUT)
        
        self.srv = self.create_service(ResetBoards, 'reset_boards', self.reset_board_callback)

    def reset_board_callback(self, request, response):
        
        self.reset_pin.set_line(1)        
        
        time.sleep(5)
        
        self.reset_pin.set_line(0) 
        
        response.ok = not self.reset_pin.get_value()

        return response
    
    def __del__(self):
        self.reset_pin.release()

def main(args=None):
    rclpy.init(args=args)

    service = ResetBoardsService()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()