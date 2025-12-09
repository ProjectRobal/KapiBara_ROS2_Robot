from kapibara.devices.Device import Device
import kapibara.DeviceExceptions as errors
import board
import busio
import adafruit_vl53l0x

from sensor_msgs.msg import Range

class DistanceSensor(Device):
    def __init__(self,node,time,name,id,port):
        super().__init__(node,time,name,id,port)
        self.init()
        
    def init(self):
        try:
            self.SelectPort()
            self._i2c= busio.I2C(board.SCL, board.SDA)
            self._sensor=adafruit_vl53l0x.VL53L0X(self._i2c)
            #self._publisher = self._node.create_publisher(Range, self._name, 10)
            #self._timer=self._node.create_timer(self._time, self.callback)
        except:
            raise errors.DeviceInitError(self.name(),"Cannot connect to I2C")
    
    def start(self):
        self._publisher = self._node.create_publisher(Range, self._name, 10)
        self._timer=self._node.create_timer(self._time, self.callback)
    
    def callback(self):
        
        self.SelectPort()
        
        try:
            _range=Range()
            _range.radiation_type=1
            _range.field_of_view=0.1745
            _range.min_range=0.0
            _range.max_range=1.0
            _range.range=float(self._sensor.range)/1000.0
            
            self._publisher.publish(_range)

        except:
            raise errors.DeviceGeneralFault(self.name(),"Cannot read distance data")



