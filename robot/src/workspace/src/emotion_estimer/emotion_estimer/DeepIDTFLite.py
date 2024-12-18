import cv2


import tensorflow as tf
import numpy as np

import platform

try:
    from tflite_runtime.interpreter import Interpreter
    from tflite_runtime.interpreter import load_delegate
except:
    from tensorflow.lite.python.interpreter import Interpreter
    from tensorflow.lite.python.interpreter import load_delegate
    
from ament_index_python.packages import get_package_share_directory

EDGETPU_SHARED_LIB = {
  'Linux': 'libedgetpu.so.1',
  'Darwin': 'libedgetpu.1.dylib',
  'Windows': 'edgetpu.dll'
}[platform.system()]

class DeepIDTFLite:
    def __init__(self,filepath):
        # tflite model init
        self._interpreter = tf.lite.Interpreter(model_path=filepath,
                                                experimental_delegates=[load_delegate(EDGETPU_SHARED_LIB)])
        self._interpreter.allocate_tensors()

        # model details
        input_details = self._interpreter.get_input_details()
        output_details = self._interpreter.get_output_details()
        
        self.width = input_details[0]['shape'][1]
        self.height = input_details[0]['shape'][2]
        
        self.input_index = input_details[0]['index']
        self.output_index = output_details[0]['index']
        
        print(input_details[0]['shape'])
        print(output_details[0]['shape'])
        
    def process(self,img):
        
        img = cv2.resize(img,(self.width,self.height))
        img = cv2.normalize(img,None,alpha=0.0,beta=1.0,norm_type=cv2.NORM_MINMAX)
        img = img.astype(np.float32)
        img = np.transpose(img,(1,0,2))
        
        img = img[None, ...]
        
        self._interpreter.set_tensor(self.input_index,img)
        self._interpreter.invoke()
        
        embeddings = self._interpreter.get_tensor(self.output_index)
        
        return embeddings
        
        