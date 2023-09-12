import onnx
from onnx import parser
from onnx import checker
import onnxruntime as ort
import numpy as np
import sys
from interfaces.srv import TrackingData
from interfaces.srv import CarAction
from std_msgs.msg import Float32MultiArray
import rclpy
import os
import cv2
from rclpy.node import Node

class Connection(Node):

	def __init__(self):
		super().__init__('connection')
		# self.clientTracking = self.create_client(TrackingData, 'tracking')
		# while not self.clientTracking.wait_for_service(timeout_sec=1.0):
		# 	self.get_logger().info('service not available, waiting again...')
		# self.reqTracking = TrackingData.Request()
		self.subscription = self.create_subscription(
			Float32MultiArray, 
			'car_video_frames', 
			self.car_frame_callback, 
			10)
		self.subscription # prevent unused variable warning
		self.reqActions = CarAction.Request()

		# self.clientActions = self.create_client(CarAction, 'action')
		# while not self.clientActions.wait_for_service(timeout_sec=1.0):
		# 	self.get_logger().info('service not available, waiting again...')
		# self.reqActions = CarAction.Request()

		self.name = "model/stack_cnn.onnx"
		self.path = os.path.join(os.path.dirname(__file__), self.name)	
		self.net = onnx.load(self.path)

		self.net = onnx.version_converter.convert_version(self.net, 18)
		self.net.ir_version = 8
		checker.check_model(self.net)
		self.session = ort.InferenceSession(self.net.SerializeToString())

		self.curr_frame_stack = np.zeros((1, 9, 80, 60), dtype=np.float32)
		self.frame_stack_buffer = []

		self.img_threshold = 150

		self.net_timer = self.create_timer(0.1, self.net_callback)

	def car_frame_callback(self, msg):
		car_frame_array = np.array(msg.data)
		car_frame = car_frame_array.reshape(60,80,3)
		#normalize image
		# transforms.Normalize(mean=[0.485, 0.456, 0.406, 0.485, 0.456, 0.406, 0.485, 0.456, 0.406],
		# 						std=[0.229, 0.224, 0.225, 0.229, 0.224, 0.225, 0.229, 0.224, 0.225])	

		mean = np.array([0.485, 0.456, 0.406])
		std = np.array([0.229, 0.224, 0.225])
		image_normalized = (car_frame - mean) / std
		image_normalized = image_normalized.astype(np.float32)


		#add image to buffer
		self.frame_stack_buffer.append(image_normalized)
		if(len(self.frame_stack_buffer) == 3):
			#construct (1, 9, 60, 80) from buffer (3, 3, 60, 80)
			self.curr_frame_stack = np.stack(self.frame_stack_buffer, axis=0).reshape(1, 9, 80, 60)
			# first_image = self.curr_frame_stack[0][0:3]
			# #rearrage channels (3, 60, 80) -> (60, 80, 3)
			# first_image = first_image.reshape(60, 80, 3)

			# second_image = self.curr_frame_stack[0][3:6]
			# second_image = second_image.reshape(60, 80, 3)

			# #show image
			# cv2.imshow('frame', second_image.astype('uint8'))
			# cv2.waitKey(1)

			self.frame_stack_buffer = []

		#show first image in buffer

	
	# def get_observations(self):
	# 	self.future = self.clientTracking.call_async(self.reqTracking)
	# 	rclpy.spin_until_future_complete(self, self.future)
	# 	return self.future.result()

	def send_actions(self, steer, speed):
		self.reqActions.steer = steer + 0.075
		self.reqActions.speed = np.interp(speed, [0, 1], [0, 23000])

		self.future = self.clientActions.call_async(self.reqActions)
		# rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()

	def net_callback(self):
		ort_inputs = {self.session.get_inputs()[0].name: self.curr_frame_stack}
		act = self.session.run(None, ort_inputs)[0][0]
		print(act)

		# self.send_actions(act[0].item(), act[1].item())
		

	def data_transform(self):
		#Todo get_observations aufrufen und transformieren		
		pass

def main(args=None):
	rclpy.init(args=args)

	connection_node = Connection()

	rclpy.spin(connection_node)
	
	connection_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
