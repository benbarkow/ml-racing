import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
#float multi array
from std_msgs.msg import Float32MultiArray
# from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import onnxruntime as ort
import os
import onnx
from onnx import checker
import torch
import time
 
class CarCameraPublisher(Node):
	def __init__(self):
		super().__init__('car_camera_pub')
		self.publisher_ = self.create_publisher(Float32MultiArray, 'car_video_features', 10)
		timer_period = 0.00  # seconds
		self.timer = self.create_timer(timer_period, self.cnn_timer_callback)
		self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
		self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
		# self.cap.set(cv2.CAP_PROP_FPS, 10)
		self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 80)
		self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 60)


		self.cnn_name = "model/stack_cnn_v2.onnx"
		self.cnn_path = os.path.join(os.path.dirname(__file__), self.cnn_name)	
		self.cnn_net = onnx.load(self.cnn_path)

		self.cnn_net = onnx.version_converter.convert_version(self.cnn_net, 18)
		self.cnn_net.ir_version = 8
		checker.check_model(self.cnn_net)
		self.cnn_session = ort.InferenceSession(self.cnn_net.SerializeToString(), providers=['CPUExecutionProvider'])
		self.image_buffer = []

		#measure publish rate / second every 5 seconds
		self.pub_counter = 0
		self.pub_rate_timer = self.create_timer(5, self.pub_rate_callback)

		#create recordings directory with date and time as name
		self.recording_dir = os.path.join(os.path.dirname(__file__), 'recordings')
		folder_name = time.strftime("%Y%m%d-%H%M%S")
		self.recording_dir = os.path.join(self.recording_dir, folder_name)
		os.mkdir(self.recording_dir)

		self.file_counter = 0

	def pub_rate_callback(self):
		self.get_logger().info('Publish Rate: {} Hz'.format(self.pub_counter/5))
		self.pub_counter = 0

	def save_image_stack(self, stack):
		#create 270x60x3 image from stack
		image1 = stack[:, :, 0:3]
		image2 = stack[:, :, 3:6]
		image3 = stack[:, :, 6:9]

		stack = np.concatenate((image1, image2, image3), axis=1)
		stack = stack.astype('uint8')
		
		image_name = str(self.file_counter) + ".png"
		image_path = os.path.join(self.recording_dir, image_name)
		cv2.imwrite(image_path, stack)
		self.file_counter += 1

	def cnn_timer_callback(self):
		ret, frame = self.cap.read()
		if ret == True:
			image = cv2.resize(frame, (80,60))
			image = image.astype('float32')
			if(len(self.image_buffer) == 3):
				stack = np.concatenate(self.image_buffer, axis=2)
				stack = np.transpose(stack, (0,1,2))
				features = self.cnn_forward(stack)
				# print(features)
				#publish features
				msg = Float32MultiArray()
				msg.data = features
				self.publisher_.publish(msg)
				self.image_buffer = []
				self.pub_counter += 1
			else:
				self.image_buffer.append(image)

	def cnn_forward(self, stack):
		self.save_image_stack(stack)

		mean = np.array([0.4635, 0.4754, 0.4603, 0.4639, 0.4757, 0.4607, 0.4644, 0.4762, 0.4611])
		std = np.array([0.1301, 0.1243, 0.1246, 0.1297, 0.1239, 0.1242, 0.1294, 0.1235, 0.1237])
		stack = ((stack/255 - mean) / std).astype('float32')
		stack = np.transpose(stack, (2,1,0))
		stack = np.expand_dims(stack, axis=0)

		ort_inputs = {self.cnn_session.get_inputs()[0].name: stack}
		image_features = self.cnn_session.run(None, ort_inputs)[0][0]


		return image_features.astype('float32').tolist()

  
def main(args=None):
  
  rclpy.init(args=args)
  camera_publisher = CarCameraPublisher()
  rclpy.spin(camera_publisher)
  camera_publisher.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()