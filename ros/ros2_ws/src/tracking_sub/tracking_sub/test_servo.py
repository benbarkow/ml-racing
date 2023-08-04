
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
import time


class Action(Node):

	def __init__(self):
		super().__init__('test_servo')
		self.publisher_steering = self.create_publisher(Float64, '/vesc/commands/servo/unsmoothed_position', 10)
		self.publisher_speed = self.create_publisher(Float64, '/vesc/commands/motor/unsmoothed_speed', 10)
		self.timer = self.create_timer(0.5, self.timer_callback)
		self.i = 0.5
		self.a = 0.05

	def timer_callback(self):
		self.i += self.a
		if self.i >= 0.8 or self.i <= 0.2:
			self.a = -self.a
		print(self.i)
		self.send(self.i, 0.0)
		if self.i == 0.45:
			time.sleep(10)
	
	def send(self,steering, speed):
		msg = Float64()
		msg.data = steering
		self.publisher_steering.publish(msg)
		msg.data = speed
		self.publisher_speed.publish(msg)

def main(args=None):
	rclpy.init(args=args)

	action = Action()

	rclpy.spin(action)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	action.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
