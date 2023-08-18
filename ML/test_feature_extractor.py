import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

#read image from test_images/* dir

def read_image(image_name):
	img = cv2.imread(image_name)
	img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	return img

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

def display_images(images, cmap=None):
	fig = plt.figure(figsize=(12, 10))
	for i, image in enumerate(images):
		fig.add_subplot(2, 3, i+1)
		plt.imshow(image, cmap)
	plt.show()

for file in os.listdir("test_images/"):
	if file.endswith(".png"):
		image = read_image("test_images/"+file)
		thresh = otsu_thresholding(image)
		canny_img = canny(image)
		warped, Minv = warp(image)
		thresh, Minv = warp(thresh)
		canny_img, Minv = warp(canny_img)
		kernel = np.ones((15,15), np.uint8)
		canny_img = cv2.dilate(canny_img, kernel, iterations=1)


		intersection = cv2.bitwise_and(thresh, canny_img)
		display_images([image, warped, thresh, canny_img, intersection], cmap='gray')
		#image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		#show normal and warped and canny





