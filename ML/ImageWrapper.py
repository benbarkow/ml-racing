import gym
import os
from gym.spaces import Box, Tuple, Dict
import numpy as np
from torchvision.models import resnet18, ResNet18_Weights
from torchvision import transforms
import torch
from PIL import Image
import time 
import cv2


class ImageWrapper(gym.ObservationWrapper):
	def __init__(self, env):
		super().__init__(env)
		# self.observation_space = Box(shape=(60,80,), low=0, high=255, dtype=np.uint8)
		# self.observation_space = Tuple((Box(shape=(60,80,), low=0, high=255, dtype=np.uint8), Box(shape=(2,), low=-1, high=1, dtype=np.float32)));
		self.observation_space = Dict({
			"image": Box(shape=(60,80), low=0, high=255, dtype=np.uint8),
			# "vector": Box(shape=(2,), low=-1, high=1, dtype=np.float32)
		})
		# self.observation_space = Tuple((Box(shape=(512,), low=0, high=1, dtype=np.float32), Box(shape=(2,), low=-1, high=1, dtype=np.float32)));
	def observation(self, obs):
		# rescale the image from 0-1 to 0-255 and convert to uint8

		image = obs[0] * 255
		image = image.astype(np.uint8)

		# image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
		# #gaussian blur
		# image = cv2.GaussianBlur(image, (5,5), 0)
		# #apply threshold
		# _, image = cv2.threshold(image, 100, 255, cv2.THRESH_BINARY)
		# #convert to grayscale
		# image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		#resize
		image = cv2.resize(image, (80,60))
		# #display image
		# cv2.imshow("image", image)
		# cv2.waitKey(1)

		return {"image": image}