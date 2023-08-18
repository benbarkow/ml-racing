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

def canny(image):
	#convert to grayscale
	gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	#apply gaussian blur
	blur = cv2.GaussianBlur(gray, (5,5), 0)
	#apply canny edge detection
	canny = cv2.Canny(blur, 50, 150)
	#dialate image
	kernel = np.ones((5,5), np.uint8)
	# canny = cv2.dilate(canny, kernel, iterations=1)
	return canny

def otsu_thresholding(image):
	#convert to grayscale
	gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	#apply gaussian blur
	blur = cv2.GaussianBlur(gray, (5,5), 0)
	#apply otsu thresholding
	ret, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	return thresh

def warp(image):
	#image size
	height = image.shape[0]
	width = image.shape[1]
	roi = np.array([[(0, 0), (width, 0), (width*3, height/2), (-width*2, height/2)]], dtype=np.int32)

	#src points
	src = np.float32(roi)
	#dst points
	dst = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
	
	#perspective transform matrix
	M = cv2.getPerspectiveTransform(src, dst)
	#inverse perspective transform matrix
	Minv = cv2.getPerspectiveTransform(dst, src)
	#warped image
	warped = cv2.warpPerspective(image, M, (width, height), flags=cv2.INTER_LINEAR)
	return warped, Minv

class ImageWrapper(gym.ObservationWrapper):
	def __init__(self, env):
		super().__init__(env)
		# self.observation_space = Box(shape=(60,80,), low=0, high=255, dtype=np.uint8)
		# self.observation_space = Tuple((Box(shape=(60,80,), low=0, high=255, dtype=np.uint8), Box(shape=(2,), low=-1, high=1, dtype=np.float32)));
		self.observation_space = Dict({
			"image": Box(shape=(256,342, 1), low=0, high=255, dtype=np.uint8),
		})
		# self.observation_space = Tuple((Box(shape=(512,), low=0, high=1, dtype=np.float32), Box(shape=(2,), low=-1, high=1, dtype=np.float32)));
	def observation(self, obs):
		#obs[0] is image rgb
		image = obs[0]
		#to uint8
		image = (image * 255).astype(np.uint8)

		thresh = otsu_thresholding(image)
		canny_img = canny(image)
		warped, Minv = warp(image)
		thresh, Minv = warp(thresh)
		canny_img, Minv = warp(canny_img)
		kernel = np.ones((15,15), np.uint8)
		canny_img = cv2.dilate(canny_img, kernel, iterations=1)
		intersection = cv2.bitwise_and(thresh, canny_img)
		#print datatype of image
		#convert to (256, 342, 1)
		intersection = np.expand_dims(intersection, axis=2)

		#display intersection
		# cv2.imshow("intersection", intersection)
		# cv2.waitKey(1)

		# image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
		# #gaussian blur
		# image = cv2.GaussianBlur(image, (5,5), 0)
		# #apply threshold
		# _, image = cv2.threshold(image, 100, 255, cv2.THRESH_BINARY)
		# #convert to grayscale
		# image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		#resize
		# image = cv2.resize(image, (80,60, 1))
		# #display image
		# cv2.imshow("image", image)
		# cv2.waitKey(1)
		# print(is_image_space(self.observation_space["image"], check_channels=False))

		return {"image": intersection}

	