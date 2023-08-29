import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import time

class Tracking(Node):
	def __init__(self):
		super().__init__('tracking_node')
		self.subscriptionPosition = self.create_subscription(
			PoseStamped,
			'vrpn_client_node/cf_jp_ma/pose',
			self.listener_callback,
			10)
		
		self.subscriptionPosition  # prevent unused variable warning
		self.trackingService = self.create_service(
			Float32MultiArray,
			'tracking',
			self.tracking_callback)
	
		self.prevPose = None
		self.lastListen = time.time()
		self.speedX = 0.0
		self.speedY = 0.0
		self.position = [0.0,0.0]
	
	def tracking_callback(self, request, response):
		response.speedx = self.speedX
		response.speedy = self.speedY
		response.positioncar = self.position

	def listener_callback(self, msg):
		if(time.time() - self.lastListen < 0.1):
			#return
			pass
		# Hier anstatt z y nehmen?
		self.speedX, self.speedY = self.speed_from_positions(self.prevPose, msg)
		x = msg.pose.position.x
		y = msg.pose.position.y
		self.position = [x,y]

		forward_vector_car = self.quaternion_to_forward_vector(msg)

		self.prevPose = msg
		self.lastListen = time.time()

	def speed_from_positions(self, firstPose, secondPose):
		if(firstPose == None):
			return (0,0)		
		t1 = firstPose.header.stamp.nanosec/pow(10,8)
		t2 = secondPose.header.stamp.nanosec/pow(10,8)
		deltaT = t2 - t1
			
		x1 = firstPose.pose.position.x
		x2 = secondPose.pose.position.x			
		deltaX = x2 - x1

		y1 = firstPose.pose.position.y
		y2 = secondPose.pose.position.y
		
		deltaY = y2 - y1
		speedX = deltaX/deltaT
		speedY = deltaY/deltaT
		return (speedX, speedY)
		

	def quaternion_to_forward_vector(self, msg):
		quaternion = msg.pose.orientation
		x = quaternion.x
		y = quaternion.y
		z = quaternion.z
		w = quaternion.w

		forward_x  = 2 *(x*z + w*y)
		forward_y = 2 * (y*z - w*x)
		forward_z = 1 - 2 * (x*x + y*y)

		forward_z_left = - forward_z
		
		return (forward_x, forward_y, forward_z_left)

	def angle_from_forward_vectors(forward_car, forward_parking):
		angleRad = np.arccos(np.dot(forward_car, forward_parking))
		angle = np.rad2deg(angleRad)
		print(angle)

		
def main(args=None):
	rclpy.init(args=args)

	tracking_node= Tracking()
	
	rclpy.spin(tracking_node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	tracking_node.destroy_node()
	#position_subscriber_park.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
