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
from stable_baselines3.common.preprocessing import is_image_space, is_image_space_channels_first
from stack_model import CNN

class StackCnnWrapper(gym.ObservationWrapper):
	def __init__(self, env):
		super().__init__(env)
		# self.observation_space = Box(shape=(60,80,), low=0, high=255, dtype=np.uint8)
		# self.observation_space = Tuple((Box(shape=(60,80,), low=0, high=255, dtype=np.uint8), Box(shape=(2,), low=-1, high=1, dtype=np.float32)));
		self.observation_space = Box(shape=(8,), low=0, high=1, dtype=np.float32)
		self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
		self.model = CNN().to(self.device)
		checkpoint = torch.load('models/cnn_weights/stack_cnn_mse.pth')
		checkpoint = {k.replace('module.', ''): v for k, v in checkpoint.items()}
		self.model.load_state_dict(checkpoint)
		self.model.eval()
		# self.observation_space = Tuple((Box(shape=(512,), low=0, high=1, dtype=np.float32), Box(shape=(2,), low=-1, high=1, dtype=np.float32)));

	def observation(self, obs):
		#obs[0] is image stack (80, 60, 9)
		images = obs[0] * 255
		images = images.astype(np.uint8)

		preprocess = transforms.Compose([
			transforms.ToTensor(),
			transforms.Normalize(mean=[0.2780, 0.2813, 0.2733, 0.2788, 0.2823, 0.2744, 0.2801, 0.2840, 0.276],
							std=[0.1518, 0.1828, 0.1830, 0.1524, 0.1838, 0.1840, 0.1534, 0.1855, 0.1858])		
		])

		image_tensor = preprocess(images)
		image_tensor = image_tensor.unsqueeze(0)
		image_tensor = image_tensor.to(self.device)

		with torch.no_grad():
			output = self.model(image_tensor)
			output = output.cpu().numpy()

		output = output.reshape(8,)

		return output

	