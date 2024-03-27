import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs

class RealSenseRgbPublisher(Node):
    def __init__(self):
        super().__init__('realsense_rgb_publisher')
        self.publisher_rgb = self.create_publisher(Image, 'rgb_image', 10)
        self.publisher_depth = self.create_publisher(Image, 'depth_image', 10)
        self.bridge = CvBridge()

        # Create a timer callback to capture and publish images
        
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)

        # Start the pipeline
        self.pipeline.start(self.config)
        
        self.timer = self.create_timer(1, self.capture_and_publish)

    def capture_and_publish(self):
        #pipeline = rs.pipeline()
        #config = rs.config()
        #config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
        #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)

        #pipeline.start(config)
        try:
            # Wait for a frame
            frames = self.pipeline.wait_for_frames()

            # Get the color frame
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "mono16")
            self.publisher_rgb.publish(msg)
            self.publisher_depth.publish(depth_msg)
            self.get_logger().info('Published RGB-D image')
        
        except Exception as e:
            self.get_logger().error('Error capturing and publishing images: %s' % str(e))
        #finally:
            #pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    rgb_publisher = RealSenseRgbPublisher()
    rclpy.spin(rgb_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rgb_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
