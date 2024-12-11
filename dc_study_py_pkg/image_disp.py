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

        # Dictionary to store images by frame_id
        self.images = {}

        # Display thread
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

    def image_L_cb(self, msg):
        try:
            frame_id = msg.header.frame_id
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            image = cv2.rotate(image, cv2.ROTATE_180)
            image = cv2.flip(image, 1)

            # Store the left image with its frame_id
            if frame_id not in self.images:
                self.images[frame_id] = {'left': image}
            else:
                self.images[frame_id]['left'] = image

            self.stitch_images(frame_id)
        except Exception as e:
            self.get_logger().error(f"Error in left image callback: {e}")

    def image_R_cb(self, msg):
        try:
            frame_id = msg.header.frame_id
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            image = cv2.rotate(image, cv2.ROTATE_180)
            image = cv2.flip(image, 1)

            # Store the right image with its frame_id
            if frame_id not in self.images:
                self.images[frame_id] = {'right': image}
            else:
                self.images[frame_id]['right'] = image

            self.stitch_images(frame_id)
        except Exception as e:
            self.get_logger().error(f"Error in right image callback: {e}")

    def stitch_images(self, frame_id):
        # Proceed only if both left and right images for the same frame_id are available
        if frame_id in self.images and 'left' in self.images[frame_id] and 'right' in self.images[frame_id]:
            try:
                image_L = self.images[frame_id]['left']
                image_R = self.images[frame_id]['right']

                # Stitch images
                stitched = np.hstack((image_L, image_R))
                stitched_image = cv2.resize(stitched, (1920, 1080))

                # Display the stitched image
                self.stitched_image = stitched_image

                # Clean up to save memory
                del self.images[frame_id]
            except Exception as e:
                self.get_logger().error(f"Error during stitching: {e}")

    def display_loop(self):
        cv2.namedWindow('Stitched Image', cv2.WINDOW_NORMAL)
        fullscreen = False

        while rclpy.ok():
            if hasattr(self, 'stitched_image') and self.stitched_image is not None:
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
