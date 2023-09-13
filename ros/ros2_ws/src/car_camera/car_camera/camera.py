import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
#float multi array
from std_msgs.msg import Float32MultiArray
# from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
 
class CarCameraPublisher(Node):
	def __init__(self):
		super().__init__('car_camera_pub')
		self.publisher_ = self.create_publisher(Float32MultiArray, 'car_video_frames', 10)
		timer_period = 2.00  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.cap = cv2.VideoCapture(0)
		self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
		self.cap.set( cv2.CAP_PROP_FPS, 5)

		#measure publish rate / second every 5 seconds
		self.pub_counter = 0
		self.pub_rate_timer = self.create_timer(5, self.pub_rate_callback)

	def pub_rate_callback(self):
		self.get_logger().info('Publish Rate: {} Hz'.format(self.pub_counter/5))
		self.pub_counter = 0
	
	def timer_callback(self):
		ret, frame = self.cap.read()
		#resize image to 320x240
		if ret == True:
			#read image as rgb
			image = cv2.resize(frame, (80,60))
			#convert to float32
			image = image.astype('float32')

			image_array = image.reshape(-1).tolist()
			#publish image
			msg = Float32MultiArray()
			msg.data = image_array
			self.publisher_.publish(msg)
			self.pub_counter += 1
		else:
			self.get_logger().info('Cannot read camera frame')

  
def main(args=None):
  
  rclpy.init(args=args)
  camera_publisher = CarCameraPublisher()
  rclpy.spin(camera_publisher)
  camera_publisher.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()