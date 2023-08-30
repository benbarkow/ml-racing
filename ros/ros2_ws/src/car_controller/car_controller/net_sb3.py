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

		self.clientActions = self.create_client(CarAction, 'action')
		while not self.clientActions.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		self.reqActions = CarAction.Request()

		self.name = "model/working_model.onnx"
		self.path = os.path.join(os.path.dirname(__file__), self.name)	
		self.net = onnx.load(self.path)

		self.net = onnx.version_converter.convert_version(self.net, 18)
		self.net.ir_version = 8
		checker.check_model(self.net)
		self.session = ort.InferenceSession(self.net.SerializeToString())
		self.curr_frame = np.zeros((1, 4800)).astype(np.float32)

		self.img_threshold = 150

		self.net_timer = self.create_timer(0.1, self.net_callback)

	def car_frame_callback(self, msg):
		self.car_frame = np.array(msg.data)
		msg_frame = self.car_frame.reshape((60,80))
		image = msg_frame.astype(np.uint8)


		image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
		image = cv2.GaussianBlur(image, (5,5), 0)
		_, image = cv2.threshold(image, self.img_threshold, 255, cv2.THRESH_BINARY)
		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		image = cv2.resize(image, (80,60))

		cv2.imshow('frame', image.astype('uint8'))
		cv2.waitKey(1)

		image = image.astype(np.float32)
		image = image.reshape((1, 4800))
		#show image
		self.curr_frame = image
	
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
		ort_inputs = {self.session.get_inputs()[0].name: self.curr_frame}
		act = self.session.run(None, ort_inputs)[0][0]
		print(act)

		self.send_actions(act[0].item(), act[1].item())
		

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
