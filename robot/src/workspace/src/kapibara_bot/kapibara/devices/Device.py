import smbus2 as smbus
import kapibara.DeviceExceptions as errors
import rclpy
from rclpy.node import Node

class Device:
    
    def __init__(self,node,time,name,id,port):
        self._time=time
        self._node=node
        self._name=name
        self._id=id
        self._port=port

    def SelectPort(self):

        if self._port <0 and self._port > 7:
            return
        try:
        
            bus=smbus.SMBus(1)

            bus.write_byte_data(0x70,0,1<<self._port)

            bus.close()
        except:
            raise errors.HubError("Cannot switch to channel: "+str(self._port))

    def start(self):
        raise NotImplementedError()
    
    def port(self):
        return self._port

    def id(self):
        return self._id

    def name(self):
        return self._name