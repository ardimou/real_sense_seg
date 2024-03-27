import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class SegmentedImageSubscriber(Node):
    def __init__(self):
        super().__init__('segmented_image_subscriber')
        self.subscription = self.create_subscription(Image, 'segmentation_image', self.image_callback, 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info("Segmenting image")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #timestamp = rclpy.clock.Clock().now().nanoseconds  # Use timestamp for unique file name
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"segmented_images/{timestamp}.jpg"  # Adjust the file format as needed
        directory = '/path/to/save/directory'  # Specify the directory where you want to save the images
        file_path = os.path.join(directory, filename)
        cv2.imwrite(file_path, cv_image)
        

def main(args=None):
    rclpy.init(args=args)
    segmented_image_subscriber = SegmentedImageSubscriber()
    rclpy.spin(segmented_image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()