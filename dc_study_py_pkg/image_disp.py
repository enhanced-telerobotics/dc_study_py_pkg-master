import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading


class ImageStitchAndDisplayNode(Node):
    def __init__(self):
        super().__init__('image_stitch_and_display')
        self.bridge = CvBridge()

        # Subscribers
        self.image_L_sub = self.create_subscription(
            Image, '/cop_vision_left', self.image_L_cb, 10
        )
        self.image_R_sub = self.create_subscription(
            Image, '/cop_vision_right', self.image_R_cb, 10
        )

        # Image placeholders
        self.image_L = None
        self.image_R = None
        self.stitched_image = None

        # Display thread
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

    def image_L_cb(self, msg):
        try:
            self.image_L = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_L = cv2.rotate(self.image_L, cv2.ROTATE_180)
            self.image_L = cv2.flip(self.image_L, 1)
            self.stitch_images()
        except Exception as e:
            self.get_logger().error(f"Error in left image callback: {e}")

    def image_R_cb(self, msg):
        try:
            self.image_R = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_R = cv2.rotate(self.image_R, cv2.ROTATE_180)
            self.image_R = cv2.flip(self.image_R, 1)
            self.stitch_images()
        except Exception as e:
            self.get_logger().error(f"Error in right image callback: {e}")

    def stitch_images(self):
        if self.image_L is not None and self.image_R is not None:
            try:
                # Stitch images
                stitched = np.hstack((self.image_L, self.image_R))
                self.stitched_image = cv2.resize(stitched, (1920, 1080))
            except Exception as e:
                self.get_logger().error(f"Error during stitching: {e}")

    def display_loop(self):
        cv2.namedWindow('Stitched Image', cv2.WINDOW_NORMAL)
        fullscreen = False

        while rclpy.ok():
            if self.stitched_image is not None:
                cv2.imshow('Stitched Image', self.stitched_image)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('f'):
                fullscreen = not fullscreen
                cv2.setWindowProperty(
                    'Stitched Image', cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL
                )
            elif key == 27:  # ESC key
                break

        cv2.destroyAllWindows()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageStitchAndDisplayNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
