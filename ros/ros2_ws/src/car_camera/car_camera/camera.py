import cv2
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class CameraPublisher(Node):

	def __init__(self):
		super().__init__('camera_publisher')
		self.publisher_ = self.create_publisher(String, 'car_camera', 10)
		timer_period = 0.5  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		self.cap = cv2.VideoCapture(0)

	def timer_callback(self):
		# Capture frame-by-frame
		ret, frame = self.cap.read()
		# convert frame to float32[]
		frame = frame.astype('float32')
		data = frame.flatten()
		# Publish the data
		msg = Float32MultiArray()
		msg.data = data
		self.publisher_.publish(msg)
		self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()