# pip3 install adafruit-circuitpython-pca9685
from kapibara.devices.Device import Device
import kapibara.DeviceExceptions as errors
from kapibara.libs.adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from board import SCL, SDA
import busio
import math

from kapibara_interfaces.msg import Servo
from std_msgs.msg import Float64MultiArray

class PWM:
    def __init__(self,off):
        self.off = off
    def __str__(self):
        return str({ "duty cycle": self.off})

class Servos(Device):
    def __init__(self,node,time,name,id,port):
        super().__init__(node,time,name,id,port)
        self._offsets=[0.0]*16
        self._servos=[servo.Servo]*16
        self.init()
        
    def init(self):
        try:
            self.SelectPort()
            self.i2c=busio.I2C(SCL,SDA)
            self.pda=PCA9685(self.i2c)
            self.set_frequency(50)

            for i in range(16):
                self._servos[i]=servo.Servo(self.pda.channels[i])

        except:
            raise errors.DeviceInitError(self.name(),"Cannot connect to i2c")
        
    def callback(self,msg:Float64MultiArray):
        id=0
        
        for angle in msg.data:
            try:
                self.set_channel(id,angle*(180.0/math.pi))
                id+=1
            except ValueError:
                print("Value: ",angle)

    def set_frequency(self,frequency):
        self.SelectPort()
        self.pda.frequency=frequency

    def set_channel(self,channel,pwm):
        self.SelectPort()
        if channel < 16 and channel >= 0:
            angle=pwm+self._offsets[channel]        
            self._servos[channel].angle=abs(angle)

    def set_offset(self,channel,offset):
        if channel < 16 and channel >= 0:
            self._offsets[channel]=offset

    def start(self):
        
        self.subscription = self._node.create_subscription(
        Float64MultiArray,
        self._name,
        self.callback,
        10)