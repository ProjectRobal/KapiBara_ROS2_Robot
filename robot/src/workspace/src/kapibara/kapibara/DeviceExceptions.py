# messgae - error message, name - device name from which is originated, code - error code
class GeneralDeviceError(Exception):
    def __init__(self,name,message,code):
        self.message=message
        self.name=name
        self.code=code
        super().__init__(message)
    
    def __str__(self):
        return str({"name":self.name,"message":self.message,"code":self.code})


# no device has been found
class NoDeviceFound(GeneralDeviceError):
    def __init__(self,name,message):
        super().__init__(message,name,-1)

# device cannot connect to
class DeviceConnectionError(GeneralDeviceError):
    def __init__(self,name,message):
        super().__init__(message,name,-2)



# wrong device parameter was given
class WrongDeviceParameter(GeneralDeviceError):
    def __init__(self,name,message):
        super().__init__(message,name,-3)

# wrong device parameter was given
class HubError(GeneralDeviceError):
    def __init__(self,message):
        super().__init__(message,"TCA9548",-4)

# device suddenly stopped working
class DeviceGeneralFault(GeneralDeviceError):
    def __init__(self,name,message):
        super().__init__(message,name,-5)

# device initialization error
class DeviceInitError(GeneralDeviceError):
    def __init__(self,name,message):
        super().__init__(message,name,-6)


# a type of errors called when something very bad happened

class DeviceCriticalError(Exception):
    def __init__(self,name,message,code):
        self.message=message
        self.name=name
        self.code=code
        super().__init__(message)
    
    def __str__(self):
        return str({"name":self.name,"message":self.message,"code":self.code})