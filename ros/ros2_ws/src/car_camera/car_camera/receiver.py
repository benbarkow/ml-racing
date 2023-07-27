import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
class CarCameraReciever(Node):
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(
      Image, 
      'car_video_frames', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.br = CvBridge()
   
  def listener_callback(self, data):
    current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("car_camera", current_frame)
    cv2.waitKey(1)
  
def main(args=None):
  rclpy.init(args=args)
  car_camera_sub = CarCameraReciever()
  rclpy.spin(car_camera_sub)
  car_camera_sub.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()