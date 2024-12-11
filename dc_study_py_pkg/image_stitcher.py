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

        # Dictionaries to store images and timestamps based on frame_id
        self.image_L_dict = {}
        self.image_R_dict = {}

    def image_L_cb(self, msg):
        # Store the left image based on frame_id
        frame_id = msg.header.frame_id
        self.image_L_dict[frame_id] = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        self.try_stitch_and_publish(frame_id)

    def image_R_cb(self, msg):
        # Store the right image based on frame_id
        frame_id = msg.header.frame_id
        self.image_R_dict[frame_id] = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        self.try_stitch_and_publish(frame_id)

    def try_stitch_and_publish(self, frame_id):
        # Check if both images for the same frame_id are available
        if frame_id in self.image_L_dict and frame_id in self.image_R_dict:
            # Get the images
            image_L = self.image_L_dict.pop(frame_id)
            image_R = self.image_R_dict.pop(frame_id)

            # Process the images (e.g., rotation or flipping if necessary)
            image_L = cv2.rotate(image_L, cv2.ROTATE_180)
            image_L = cv2.flip(image_L, 1)
            image_R = cv2.rotate(image_R, cv2.ROTATE_180)
            image_R = cv2.flip(image_R, 1)

            # Stitch images horizontally
            stitched_image = np.hstack((image_L, image_R))
            stitched_image = cv2.resize(stitched_image, (1920, 1080))

            # Convert stitched image back to ROS Image message
            stitched_msg = self.bridge.cv2_to_imgmsg(stitched_image, 'rgb8')
            stitched_msg.header.frame_id = frame_id  # Use the same frame_id
            stitched_msg.header.stamp = self.get_clock().now().to_msg()  # Use the current time
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
