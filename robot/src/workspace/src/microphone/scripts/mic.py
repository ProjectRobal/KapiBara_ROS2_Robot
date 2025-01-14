#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from kapibara_interfaces.msg import Microphone

from rcl_interfaces.msg import ParameterDescriptor


import numpy as np

import pyaudio


'''

We have sensors topics that will estimate angers 

'''


class MicrophoneNode(Node):

    def __init__(self):
        super().__init__('microphone_node')
        
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 2)
        self.declare_parameter('chunk_size', 1600)
        self.declare_parameter('device_id', 1)
        
        self.mic_publisher = self.create_publisher(Microphone, "mic", 10)
        
        self.audio = pyaudio.PyAudio()
        
        rate:int = self.get_parameter('sample_rate').get_parameter_value().integer_value
        channels:int = self.get_parameter('channels').get_parameter_value().integer_value
        chunk_size:int = self.get_parameter('chunk_size').get_parameter_value().integer_value
        device_id:int = self.get_parameter('device_id').get_parameter_value().integer_value
        
        self.stream = self.audio.open(rate,channels,format=pyaudio.paInt32,input=True,frames_per_buffer=chunk_size,input_device_index=device_id,stream_callback=self.callback)
        
    def callback(self,in_data, frame_count, time_info, status):
        
        output = Microphone()
        
        audio = np.frombuffer(in_data,dtype=np.int32)
        
        left = audio[::2]
        right = audio[1::2]
        
        output.channel1 = left.tolist()
        output.channel2 = right.tolist()
        output.buffor_size = frame_count
        
        self.mic_publisher.publish(output)
        
        return (in_data, pyaudio.paContinue)


def main(args=None):
    rclpy.init(args=args)

    mic_node = MicrophoneNode()

    rclpy.spin(mic_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mic_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()