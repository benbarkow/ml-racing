import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class Action(Node):

	def __init__(self):
		super().__init__('car_publisher')
		self.publisher_steering = self.create_publisher(Float64, '/vesc/commands/servo/position', 10)
		self.publisher_speed = self.create_publisher(Float64, '/vesc/commands/motor/speed', 10)
		self.actionService = self.create_service(
			Float32MultiArray,
			'action',
			self.action_callback)
		
		
	def action_callback(self, request, response):
		self.send(request.steer, request.speed)
	
	def send(self,steering, speed):
		msg = Float64()
		msg.data = steering
		self.publisher_steering.publish(msg)
		msg.data = speed
		self.publisher_speed(msg)

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
