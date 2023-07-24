import gym
import os
from gym.spaces import Box, Tuple, Dict
import numpy as np
from torchvision.models import resnet18, ResNet18_Weights
from torchvision import transforms
import torch
from PIL import Image
import time 

class ImageWrapper(gym.ObservationWrapper):
	def __init__(self, env):
		super().__init__(env)
		# self.observation_space = Box(shape=(60,80,), low=0, high=255, dtype=np.uint8)
		# self.observation_space = Tuple((Box(shape=(60,80,), low=0, high=255, dtype=np.uint8), Box(shape=(2,), low=-1, high=1, dtype=np.float32)));
		self.observation_space = Dict({
			"image1": Box(shape=(60,80,), low=0, high=255, dtype=np.uint8),
			"vector": Box(shape=(2,), low=-1, high=1, dtype=np.float32)
		})
		# self.observation_space = Tuple((Box(shape=(512,), low=0, high=1, dtype=np.float32), Box(shape=(2,), low=-1, high=1, dtype=np.float32)));
	def observation(self, obs):
		# print("Observation received:", obs)
		image = obs[0]
		# rescale the image from 0-1 to 0-255 and convert to uint8
		image = image * 255
		image = image.astype(np.uint8)
		#shape (60,80,1) to (60,80)
		image = image.reshape((60,80))

		return {"image": image, "vector": obs[1]}

