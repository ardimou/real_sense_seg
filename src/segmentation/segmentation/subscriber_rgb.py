import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class RealSenseRgbSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_rgb_subscriber')
        self.subscription_rgb = self.create_subscription(Image, 'rgb_image', self.callback, 10)
        self.subscription_depth = self.create_subscription(Image, 'depth_image', self.depth_callback, 10)
        self.bridge = CvBridge()

    def callback(self, msg):
        self.get_logger().info('Receiving rgb frame')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"./dataset/rgb_{timestamp}.jpg"
        cv2.imwrite(filename, cv_image)
        #cv2.imshow('RealSense RGB Image', cv_image)
        #cv2.waitKey(1)
    
    def depth_callback(self, msg):
        self.get_logger().info('Receiving depth frame')
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"./dataset/depth_{timestamp}.jpg"
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
        cv2.imwrite(filename, cv_image)
        
        
def main(args=None):
    rclpy.init(args=args)
    rgbd_subscriber = RealSenseRgbSubscriber()
    try:
        rclpy.spin(rgbd_subscriber)
    finally:
        rgbd_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
