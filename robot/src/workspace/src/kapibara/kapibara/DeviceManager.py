from devices.Device import Device
import RPi.GPIO as GPIO
import smbus2 as smbus
import DeviceExceptions as errors

# Example is using TCA9548A I2C multiplexer
#   Devices connected to I2C
#   name    port_number
#   Gyroscope 0
#   PWM 1
#   Sensor Front 2
#   Sensor Down 3

class DeviceManager:

    # pins - list with pins used by i2c multiplexer
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.devs={}
        

    def addDevice(self,device):
        if isinstance(device,Device):
            self.devs[device.name()] = device
            self.SelectPort(self.devs[device.name()].port())
            device.start()

    def SelectPort(self,number):

        if number <0 and number > 7:
            return

        try:
        
            bus=smbus.SMBus(1)

            bus.write_byte_data(0x70,0,1<<number)

            bus.close()
        except:
            raise errors.HubError("Cannot switch to channel: "+str(number))

    
    def loop(self):

        for dev in self.devs.values():

            self.SelectPort(dev.port())

            dev.download()

            dev.process()

            dev.upload()

    def getAllParamsList(self):
        output = {}

        for dev in self.devs.values():
            output[dev.name()] = dev.read()

        return output
    
    def getParamsList(self,name):
        output = {}

        if name in self.devs:
            output=self.devs[name].read()

        return output

    def UpdateParams(self,name,params):
        if name in self.devs:
            self.devs[name].write(params)
            return
        
        raise errors.DeviceNotFound(name,"Device has not been found!")

    def DetachDevice(self,name):
        if name in self.devs:
            del self.devs[name]

    def EmergencyDetach(self,name):
        if name in self.devs:
            self.devs[name].emergency()
            del self.devs[name]
        
    
    def clear(self):
        del self.devs

