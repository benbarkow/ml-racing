import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import time
import os
import json

class Logging(Node):
	def __init__(self):
		super().__init__('tracking_node')
		self.position_sub = self.create_subscription(
			PoseStamped,
			'/BaCar/pose',
			self.pose_callback,
			10)
		
		self.position_sub  # prevent unused variable warning

		self.features_sub = self.create_subscription(
			Float32MultiArray,
			'/car_video_features',
			self.features_callback,
			10)

		self.features_sub  # prevent unused variable warning

		self.action_sub = self.create_subscription(
			Float32MultiArray,
			'/car_actions',
			self.action_callback,
			10)
	
		self.logging_timer = self.create_timer(0.05, self.logging_callback)

		#all logging variables
		self.car_position = [None, None]
		self.image_features = [None] * 8
		self.action = [None,None]
		self.orientation = [None, None]

		self.car_positions = []
		self.image_features_list = []
		self.actions = []
		self.orientations = []
		self.timestamps = []

	def logging_callback(self):
		# print(self.car_position)
		# print(self.image_features)
		# print(self.action)

		self.car_positions.append(self.car_position)
		self.image_features_list.append(self.image_features)
		self.actions.append(self.action)
		self.orientations.append(self.orientation)
		self.timestamps.append(time.time())

	def final_logging(self):
		#create json file
		data = []

		for i in range(len(self.timestamps)):
			data.append({
				'timestamp': self.timestamps[i],
				'car_position': self.car_positions[i],
				'image_features': self.image_features_list[i],
				'action': self.actions[i],
				'orientation': self.orientations[i]
			})

		#file name is current date and time
		file_name = time.strftime("%Y%m%d-%H%M%S") + ".json"
		file_path = os.path.join(os.path.dirname(__file__), "logs/" + file_name)

		with open(file_path, 'w') as outfile:
			json.dump(data, outfile)
	
	def pose_callback(self, msg):
		x = msg.pose.position.x
		y = msg.pose.position.y
		if(x == 0 and y == 0):
			self.car_position = [None, None]
			return
		self.car_position = [x,y]

		vector = self.quaternion_to_forward_vector(msg)

		self.orientation = [vector[0], vector[1]]
		# forward_vector_car = self.quaternion_to_forward_vector(msg)

	def features_callback(self, msg):
		self.image_features = msg.data.tolist()

	def action_callback(self, msg):
		self.action[0] = msg.data[0]
		self.action[1] = msg.data[1]
		
	def quaternion_to_forward_vector(self, msg):
		quaternion = msg.pose.orientation
		x = quaternion.x
		y = quaternion.y
		z = quaternion.z
		w = quaternion.w

		# Convert the forward vector (1, 0, 0) into a quaternion format
		v = [1, 0, 0, 0]

		# Multiply the quaternion by the vector (q * v)
		qv = [
			w * v[0] + x * v[3] + y * v[2] - z * v[1],
			w * v[1] - x * v[2] + y * v[3] + z * v[0],
			w * v[2] + x * v[1] - y * v[0] + z * v[3],
			w * v[3] - x * v[0] - y * v[1] - z * v[2]
		]

		# Inverse of the quaternion
		inv_norm = 1.0 / (x * x + y * y + z * z + w * w)
		inv_q = [-x * inv_norm, -y * inv_norm, -z * inv_norm, w * inv_norm]

		# Multiply the result by the inverse of the quaternion (qv * q_inv)
		v_prime = [
			qv[3] * inv_q[0] + qv[0] * inv_q[3] + qv[1] * inv_q[2] - qv[2] * inv_q[1],
			qv[3] * inv_q[1] - qv[0] * inv_q[2] + qv[1] * inv_q[3] + qv[2] * inv_q[0],
			qv[3] * inv_q[2] + qv[0] * inv_q[1] - qv[1] * inv_q[0] + qv[2] * inv_q[3]
		]

		forward_x = v_prime[0]
		forward_y = v_prime[1]

		return [forward_x, forward_y]
		
def main(args=None):
	rclpy.init(args=args)

	logging_node = Logging()
	
	try:
		rclpy.spin(logging_node)
	except KeyboardInterrupt:
		logging_node.final_logging()

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	logging_node.destroy_node()
	#position_subscriber_park.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
