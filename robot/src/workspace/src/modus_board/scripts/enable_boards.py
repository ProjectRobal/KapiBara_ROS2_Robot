#!/usr/bin/env python3

'''
 A service used for enabling/disabling board attached to modus board.
'''

import gpiod

import rclpy
from rclpy.node import Node

from modus_board_interfaces.srv import EnableBoards


class EnableBoardsService(Node):

    def __init__(self):
        super().__init__('enable_boards_service')
        self.declare_parameter("pin",3)
        
        pin = self.get_parameter("pin").get_parameter_value().int_value
        
        self.chip = gpiod.Chip('gpiochip4')
        
        self.en_pin = self.chip.get_line(pin)
        
        self.en_pin.request(consumer="EN", type=gpiod.LINE_REQ_DIR_OUT)
        
        self.srv = self.create_service(EnableBoards, 'enable_boards', self.enable_board_callback)

    def enable_board_callback(self, request, response):
        
        n_state = request.enable
        
        self.en_pin.set_value(n_state)
        
        response.enabled = self.en_pin.get_value()

        return response
    
    def __del__(self):
        self.en_pin.release()

def main(args=None):
    rclpy.init(args=args)

    service = EnableBoardsService()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()