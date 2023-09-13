import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
#float multi array
from std_msgs.msg import Float32MultiArray
# from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import onnx
import onnxruntime as ort
import os
from onnx import checker
 
class CarCameraPublisher(Node):
	def __init__(self):
		super().__init__('car_camera_pub')
		self.publisher_ = self.create_publisher(Float32MultiArray, 'car_video_features', 10)
		timer_period = 0.00  # seconds
		self.timer = self.create_timer(timer_period, self.cnn_timer_callback)
		self.cap = cv2.VideoCapture(0)
		self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

		self.cnn_name = "model/stack_cnn.onnx"
		self.cnn_path = os.path.join(os.path.dirname(__file__), self.cnn_name)	
		self.cnn_net = onnx.load(self.cnn_path)

		self.cnn_net = onnx.version_converter.convert_version(self.cnn_net, 18)
		self.cnn_net.ir_version = 8
		checker.check_model(self.cnn_net)
		self.cnn_session = ort.InferenceSession(self.cnn_net.SerializeToString())
		self.image_buffer = []

		#measure publish rate / second every 5 seconds
		self.pub_counter = 0
		self.pub_rate_timer = self.create_timer(5, self.pub_rate_callback)

	def pub_rate_callback(self):
		self.get_logger().info('Publish Rate: {} Hz'.format(self.pub_counter/5))
		self.pub_counter = 0

	def cnn_timer_callback(self):
		ret, frame = self.cap.read()
		if ret == True:
			image = cv2.resize(frame, (80,60))
			image = image.astype('float32')
			if(len(self.image_buffer) == 3):
				stack = np.stack(self.image_buffer, axis=0).reshape(1, 9, 80, 60)			
				features = self.cnn_forward(stack)
				# print(features)
				#publish features
				msg = Float32MultiArray()
				msg.data = features
				self.publisher_.publish(msg)
				self.image_buffer = []
				self.pub_counter += 1
			else:
				self.image_buffer.append(image)

	def cnn_forward(self, stack):
		ort_inputs = {self.cnn_session.get_inputs()[0].name: stack}
		image_features = self.cnn_session.run(None, ort_inputs)[0][0]
		return image_features.astype('float32').tolist()

  
def main(args=None):
  
  rclpy.init(args=args)
  camera_publisher = CarCameraPublisher()
  rclpy.spin(camera_publisher)
  camera_publisher.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()