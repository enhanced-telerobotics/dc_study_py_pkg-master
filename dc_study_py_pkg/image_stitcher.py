import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageStitcher(Node):
    def __init__(self):
        super().__init__('image_stitcher')
        self.bridge = CvBridge()

        # Subscribers for both images
        self.image_L_sub = self.create_subscription(
            Image, 
            '/cop_vision_left', 
            self.image_L_cb, 
            10
        )
        
        self.image_R_sub = self.create_subscription(
            Image, 
            '/cop_vision_right', 
            self.image_R_cb, 
            10
        )

        # Publisher for the stitched image
        self.stitched_pub = self.create_publisher(
            Image, 
            '/stitched_image', 
            10
        )

        # Initialize image placeholders
        self.image_L = None
        self.image_R = None

    def image_L_cb(self, msg):
        # Convert ROS Image message to OpenCV image
        self.image_L = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        self.image_L = cv2.flip(self.image_L, 1)
        self.stitch_and_publish()

    def image_R_cb(self, msg):
        # Convert ROS Image message to OpenCV image
        self.image_R = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        self.image_R = cv2.flip(self.image_R, 1)
        self.stitch_and_publish()

    def stitch_and_publish(self):
        # Only proceed if both images are available
        if self.image_L is not None and self.image_R is not None:
            # Stitch images horizontally
            stitched_image = np.hstack((self.image_L, self.image_R))

            # Convert stitched image back to ROS Image message
            stitched_msg = self.bridge.cv2_to_imgmsg(stitched_image, 'rgb8')

            # Publish the stitched image
            self.stitched_pub.publish(stitched_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageStitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()