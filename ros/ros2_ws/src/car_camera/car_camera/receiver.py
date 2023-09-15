import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Float32MultiArray
import cv2 # OpenCV library
import numpy as np
 
class CarCameraReciever(Node):
	def __init__(self):
		super().__init__('image_subscriber')
		self.subscription = self.create_subscription(
		Float32MultiArray, 
		'car_video_frames', 
		self.listener_callback, 
		10)
		self.subscription # prevent unused variable warning
   
	def listener_callback(self, data):
		image_array = np.array(data.data)
		image_matrix = image_array.reshape(60,80,3)

		# print(image_matrix.shape)

		#convert to uint8
		current_frame = image_matrix.astype('uint8')

		

		#show image
		cv2.imshow('frame', current_frame.astype('uint8'))
		cv2.waitKey(1)
  
def main(args=None):
  rclpy.init(args=args)
  car_camera_sub = CarCameraReciever()
  rclpy.spin(car_camera_sub)
  car_camera_sub.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()