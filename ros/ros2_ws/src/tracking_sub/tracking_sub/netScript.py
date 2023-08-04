import onnx
from onnx import parser
from onnx import checker
import onnxruntime as ort
import numpy as np
import sys
from interfaces.srv import TrackingData
from interfaces.srv import Action
import rclpy
from rclpy.node import Node


class Connection(Node):

	def __init__(self):
		super().__init__('connection')
		self.clientTracking = self.create_client(TrackingData, 'tracking')
		while not self.clientTracking.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		self.reqTracking = TrackingData.Request()

		self.clientActions = self.create_client(Action, 'action')
		while not self.clientActions.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		self.reqActions = Action.Request()

		self.onnx_path = "rightNormalized.onnx"
		self.net = onnx.load(onnx_path)

		self.net = onnx.version_converter.convert_version(net, 18)
		self.net.ir_version = 8
		checker.check_model(self.net)
		self.session = ort.InferenceSession(self.net.SerializeToString())

	
	def get_observations(self):
		self.future = self.clientTracking.call_async(self.reqTracking)
		rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()

	def send_actions(self, steer, speed):
		self.reqActions.steer = steer
		self.reqActions.speed = speed

		self.future = self.clientActions.call_async(self.reqTracking)
		rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()

	def run(self):
		data = self.data_transform() # 	
		ort_inputs = {self.session.get_inputs()[0].name: data}
		act = session.run(None, ort_inputs)[0]
		self.send_actions(act[0], act[1])
		

	def data_transform(self):
		#Todo get_observations aufrufen und transformieren		
		pass
def main(args=None):
	rclpy.init(args=args)

	connection_node = Connection()
	
	while(True):
		connection_node.run()
	connection_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
