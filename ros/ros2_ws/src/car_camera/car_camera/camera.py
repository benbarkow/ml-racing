import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
#float multi array
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
class CarCameraPublisher(Node):
	def __init__(self):
		super().__init__('car_camera_pub')
		self.publisher_ = self.create_publisher(Float32MultiArray, 'car_video_frames', 10)
		timer_period = 0.03  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.cap = cv2.VideoCapture(0)
		self.br = CvBridge()
	
	def timer_callback(self):
		ret, frame = self.cap.read()
		#resize image to 320x240
		frame = cv2.resize(frame, (80, 60))
		if ret == True:
			image = cv2.GaussianBlur(frame, (5,5), 0)
			#apply threshold
			_, image = cv2.threshold(image, 100, 255, cv2.THRESH_BINARY)
			#convert to grayscale
			image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			#resize
			image = cv2.resize(image, (80,60))
			#convert to float32
			image = image.astype('float32')
			#publish image
			msg = Float32MultiArray()
			image_array = image.flatten().tolist()
			msg.data = image_array
			self.publisher_.publish(msg)
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