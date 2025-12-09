# a module to read data from microphones
from kapibara.devices.Device import Device
import kapibara.DeviceExceptions as errors
import pyaudio
import numpy as np

from kapibara_interfaces.msg import Microphone

AUDIODEVICENAME=""
CHUNK=32000



class Ears(Device):
    def __init__(self,node,time,name,id):
        super().__init__(node,time,name,id,99)
        
        self.channel1:list[int]=[]
        self.channel2:list[int]=[]
        
        self._size_buffor=512
        self.init()
        
    def init(self):
        try:
            
            self.audio=pyaudio.PyAudio()
            self.stream=self.audio.open(
                format=pyaudio.paInt32,
                rate=44100,
                channels=2,
                input=True,
                frames_per_buffer=32,
                stream_callback=self.callback
            )
            
        except:
            raise errors.DeviceInitError(self.name(),"Cannot start audio device")
        
    def setBufferSize(self,size:int):
        self._size_buffor=size

    def callback(self,input_data, frame_count, time_info, status_flags):

        frames=np.frombuffer(input_data,dtype=np.int32)
        i=0

        for x in frames[0::2]:
            self.channel1.append(x)
            i+=1

        i=0

        for x in frames[1::2]:
            self.channel2.append(x)
            i+=1
            
        if len(self.channel1)>self._size_buffor*2:
            self.channel1=self.channel1[-self._size_buffor:]
            
        if len(self.channel2)>self._size_buffor*2:
            self.channel2=self.channel2[-self._size_buffor:]

        return (input_data, pyaudio.paContinue)
    
    def ros_callback(self):
        
        mic=Microphone()
        
        mic.buffer_size=self._size_buffor
        
        self.channel1=self.channel1[-self._size_buffor:]
        self.channel2=self.channel2[-self._size_buffor:]

        mic.channel1=self.channel1
        mic.channel2=self.channel2
        
        self._publisher.publish(mic)
    
    def start(self):    
        self._publisher = self._node.create_publisher(Microphone, self._name, 32)
        self._timer=self._node.create_timer(self._time, self.ros_callback)
        
    def __del__(self):
        if hasattr(self,"stream"):
            self.stream.stop_stream()
            self.stream.close()