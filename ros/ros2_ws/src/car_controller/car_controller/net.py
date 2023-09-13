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

		#load rnn
		self.rnn_name = "model/track_drive_cnn_44.onnx"
		self.rnn_path = os.path.join(os.path.dirname(__file__), self.rnn_name)
		self.rnn_net = onnx.load(self.rnn_path)

		self.rnn_net = onnx.version_converter.convert_version(self.rnn_net, 18)
		self.rnn_net.ir_version = 8
		checker.check_model(self.rnn_net)
		self.rnn_session = ort.InferenceSession(self.rnn_net.SerializeToString())

		self.curr_frame_stack = np.zeros((1, 9, 80, 60), dtype=np.float32)
		self.frame_stack_buffer = []

		self.img_threshold = 150

		# self.net_timer = self.create_timer(0.1, self.net_callback)

	def car_frame_callback(self, msg):
		car_feature_array = np.array(msg.data)
		self.net(car_feature_array)


	def send_actions(self, steer, speed):
		self.reqActions.steer = steer + 0.075
		self.reqActions.speed = np.interp(speed, [0, 1], [0, 23000])

		self.future = self.clientActions.call_async(self.reqActions)
		# rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()

	def net(self, image_features):

		image_features = image_features.reshape(1, 8)
		ort_inputs = {self.rnn_session.get_inputs()[0].name: image_features}
		preds = self.rnn_session.run(None, ort_inputs)[0]

		action_dims = [11, 6]

		def softmax(x):
			"""Compute softmax values for each sets of scores in x."""
			e_x = np.exp(x - np.max(x, axis=-1, keepdims=True))
			return e_x / e_x.sum(axis=-1, keepdims=True)

		def split_and_get_argmax(logits, action_dims):
			"""Split logits based on dimensions and get argmax for each."""
			split_logits = np.split(logits, np.cumsum(action_dims)[:-1], axis=1)
			return [np.argmax(softmax(s), axis=1) for s in split_logits]


		argmax_values = split_and_get_argmax(preds, action_dims)
		actions = np.stack(argmax_values, axis=1)[0]


		print(actions)
		steering = np.interp(actions[0], [0, 10], [-1, 1])
		speed = np.interp(actions[1], [0, 5], [0, 1])
		# self.send_actions(act[0].item(), act[1].item() 
		print("steering: ", steering, "speed: ", speed)

		# self.send_actions(steering, speed)
		

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
