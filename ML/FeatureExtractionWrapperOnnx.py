import gym
import os
from gym.spaces import Box, Tuple
import numpy as np
from torchvision.models import resnet18, ResNet18_Weights
from torchvision import transforms
import torch
from PIL import Image
import onnx
from onnx import parser
from onnx import checker
import onnxruntime as ort
import numpy as np
import cv2
import time

class FeatureExtractionWrapperOnnx(gym.ObservationWrapper):
	def __init__(self, env):
		super().__init__(env)
		self.net = onnx.load("models/weights/resnet18-avgpool.onnx")
		self.observation_space = Box(shape=(514,), low=0, high=1, dtype=np.float64)
		self.image_saved = False
		self.session = ort.InferenceSession(self.net.SerializeToString(), providers=["CUDAExecutionProvider"])
		# self.observation_space = Tuple((Box(shape=(512,), low=0, high=1, dtype=np.float32), Box(shape=(2,), low=-1, high=1, dtype=np.float32)));
	def observation(self, obs):
		# rescale the image from 0-1 to 0-255 and convert to uint8
		image = obs[0] * 255
		image = image.astype(np.uint8)
		image = np.array(Image.fromarray(image, 'RGB'))
		start_time = time.time()
		preds = self.predict(image) # (1, 512)
		print("FeatureExtractionWrapperOnnx: ", time.time() - start_time)

		features = np.concatenate((preds, [obs[1]]), axis=1).astype(np.float64)
		return features
	
	def preprocess(self, img):
		img = img / 255.
		img = cv2.resize(img, (256, 256))
		h, w = img.shape[0], img.shape[1]
		y0 = (h - 224) // 2
		x0 = (w - 224) // 2
		img = img[y0 : y0+224, x0 : x0+224, :]
		img = (img - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
		img = np.transpose(img, axes=[2, 0, 1])
		img = img.astype(np.float32)
		img = np.expand_dims(img, axis=0)
		return img


	def predict(self, img):
		img = self.preprocess(img)
		ort_inputs = {self.session.get_inputs()[0].name: img}
		preds = self.session.run(None, ort_inputs)[0]
		return preds

