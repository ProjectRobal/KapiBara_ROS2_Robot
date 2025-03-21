import time
import cv2
import numpy as np
import urllib.request
import os
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

# model page: https://huggingface.co/qualcomm/Midas-V2
class midasDepthEstimator():

	def __init__(self,modelPath:str):
		self.fps = 0
		self.timeLastPrediction = time.time()
		self.frameCounter = 0

		# Initialize model
		self.initializeModel(modelPath)


	def initializeModel(self,modelPath:str):
		path = os.path.join(get_package_share_directory("emotion_estimer"),modelPath)
  
		delegates=[]
                
		if path.find("_edgetpu")>0:
			delegates=[load_delegate(EDGETPU_SHARED_LIB)]

		self.interpreter = Interpreter(
      model_path=path,
      experimental_delegates=delegates)
		self.interpreter.allocate_tensors()
  
		# Get model info
		self.getModelInputDetails()
		self.getModelOutputDetails()

	def estimateDepth(self, image):

		input_tensor = self.prepareInputForInference(image)

		# Perform inference on the image
		rawDisparity = self.inference(input_tensor)

		# Normalize and resize raw disparity
		processedDisparity = self.processRawDisparity(rawDisparity, image.shape)

		# Draw depth image
		colorizedDisparity = self.drawDepth(processedDisparity)

		# Update fps calculator
		self.updateFps()

		return colorizedDisparity

	def prepareInputForInference(self, image):
		img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		self.img_height, self.img_width, self.img_channels = img.shape

		# Input values should be from -1 to 1 with a size of 128 x 128 pixels for the fornt model
		# and 256 x 256 pixels for the back model
		img_input = cv2.resize(img, (self.inputWidth,self.inputHeight),interpolation = cv2.INTER_CUBIC)
		
		# Scale input pixel values to -1 to 1
		# mean=[0.485, 0.456, 0.406]
		# std=[0.229, 0.224, 0.225]
		# reshape_img = img_input.reshape(1, self.inputHeight, self.inputWidth,3)
		# img_input = ((img_input/ 255.0 - mean) / std).astype(np.float32)
		img_input = img_input[np.newaxis,:,:,:].astype(np.uint8)

		return img_input

	def inference(self, img_input):
		# Peform inference
		self.interpreter.set_tensor(self.input_details[0]['index'], img_input)
		self.interpreter.invoke()
		output = self.interpreter.get_tensor(self.output_details[0]['index'])
		output = output.reshape(self.outputHeight, self.outputWidth)

		return output

	def processRawDisparity(self, rawDisparity, img_shape):

		# Normalize estimated depth to have values between 0 and 255
		# depth_min = rawDisparity.min()
		# depth_max = rawDisparity.max()
		# normalizedDisparity = (255 * (rawDisparity - depth_min) / (depth_max - depth_min)).astype("uint8")
  
		# normalizedDisparity = rawDisparity

		# Resize disparity map to the sam size as the image inference
		estimatedDepth = cv2.resize(rawDisparity, (img_shape[1], img_shape[0]), interpolation=cv2.INTER_CUBIC)

		return estimatedDepth

	def drawDepth(self, processedDisparity):
		return cv2.applyColorMap(processedDisparity, cv2.COLORMAP_MAGMA)

	def getModelInputDetails(self):
		self.input_details = self.interpreter.get_input_details()
		input_shape = self.input_details[0]['shape']
		self.inputHeight = input_shape[1]
		self.inputWidth = input_shape[2]
		self.channels = input_shape[3]

	def getModelOutputDetails(self):
		self.output_details = self.interpreter.get_output_details()
		output_shape = self.output_details[0]['shape']
		self.outputHeight = output_shape[1]
		self.outputWidth = output_shape[2]	

	def updateFps(self):
		updateRate = 1
		self.frameCounter += 1

		# Every updateRate frames calculate the fps based on the ellapsed time
		if self.frameCounter == updateRate:
			timeNow = time.time()
			ellapsedTime = timeNow - self.timeLastPrediction

			self.fps = int(updateRate/ellapsedTime)
			self.frameCounter = 0
			self.timeLastPrediction = timeNow