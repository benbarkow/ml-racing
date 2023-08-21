import gym
import os
from gym.spaces import Box, Tuple
import numpy as np
from torchvision.models import resnet18, ResNet18_Weights
from torchvision import transforms
import torch
from PIL import Image
import time 

class FeatureExtractionWrapper(gym.ObservationWrapper):
	def __init__(self, env):
		super().__init__(env)
		self.model = resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)
		self.model.eval()
		self.model.cuda()
		self.layer = self.model._modules.get('avgpool')
		self.observation_space = Box(shape=(517,), low=0, high=1, dtype=np.float64)
		self.image_saved = False
		# self.observation_space = Tuple((Box(shape=(512,), low=0, high=1, dtype=np.float32), Box(shape=(2,), low=-1, high=1, dtype=np.float32)));
	def observation(self, obs):
		# rescale the image from 0-1 to 0-255 and convert to uint8
		# obs[0] is grayscale image
		image = obs[0] * 255
		image = image.astype(np.uint8)
		image = Image.fromarray(image)

		#save image in test_images folder and increment counter based on number of images in folder
		if not self.image_saved:
			print('saving image')
			image.save('test_images/' + str(len(os.listdir('test_images'))) + '.png')
			self.image_saved = True
		
		transform = transforms.Compose([
			transforms.Resize(256),
			transforms.CenterCrop(224),
			transforms.ToTensor(),
			transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
		])

		t_image = transform(image).unsqueeze(0).cuda()

		vector = torch.zeros(512)
		def copy_data(m, i, o):
			vector.copy_(o.data.reshape(o.data.size(1)))
		h = self.layer.register_forward_hook(copy_data)
		self.model(t_image)
		h.remove()

		tensor = vector
		tensor = np.concatenate((vector, obs[1]), dtype=np.float32)
		return tensor

