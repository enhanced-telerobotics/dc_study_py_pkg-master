import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Pose
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
import numpy as np
import threading
import time


class VisionSystem(Node):
    def __init__(self):
        super().__init__('vision_system')

        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.sim.setStepping(True)

        # Get vision sensor handles
        self.endscope_L = self.sim.getObject('/Vision_sensor_left')
        self.endscope_R = self.sim.getObject('/Vision_sensor_right')
        self.robot = self.sim.getObject('/ee_offset_PSM1')

        # Home position
        self.home = [-1.475, 0, 0.75, 0, np.sqrt(0.5), 0, np.sqrt(0.5)]

        # OpenCV settings
        self.window_name = 'HStack Vision Sensors'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.fullscreen = False
        self.is_start = False  # Track countdown trigger
        self.direction = np.array([0, 0, 0])  # Target direction
        # ROS 2 Subscriber
        self.robot_subscriber = self.create_subscription(
            Vector3,
            'robot_cmd',
            self.robot_callback,
            10
        )
        self.target_subscriber = self.create_subscription(
            Pose,
            'target_cmd',
            self.target_callback,
            10
        )
        self.sim.startSimulation()
        self.run_vision_loop()

    def target_callback(self, msg):
        x = msg.position.x - self.home[0]
        y = msg.position.y - self.home[1]
        z = msg.position.z - self.home[2]
        magnitude = np.sqrt(x**2 + y**2 + z**2)

        if magnitude > 0:
            self.direction = np.array([x, y, z]) / magnitude
        else:
            self.direction = np.array([0, 0, 0])

    def robot_callback(self, msg):
        """msg.x ==1 is triggered when button is first pressed before start."""
        if msg.x == 1:
            self.is_start = True

    def display_countdown(self):
        """Display a countdown before resuming the coppeliaCamera feed."""
        for text in ["Ready?", "Go"]:
            blackscreen = np.zeros((1080, 1920, 3), dtype=np.uint8)
            cv2.putText(blackscreen, text, (400, 540), cv2.FONT_HERSHEY_SIMPLEX,
                        4, (255, 255, 255), 8, cv2.LINE_AA)
            cv2.putText(blackscreen, text, (1360, 540), cv2.FONT_HERSHEY_SIMPLEX,
                        4, (255, 255, 255), 8, cv2.LINE_AA)
            cv2.imshow(self.window_name, blackscreen)
            cv2.waitKey(1000)  # Show for 2 second countdown

    def draw_progress_bar(self, img, progress, position, size=(400, 30)):
        x, y = position
        w, h = size
        red = int(255 * progress)
        green = int(255 * (1 - progress))
        color = (0, green, red)
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 255), 2)
        cv2.rectangle(
            img, (x, y), (x + int(w * (1-progress)), y + h), color, -1)

    def run_vision_loop(self):
        """Main loop for capturing and displaying images."""
        start_time = time.time()
        progress_duration = 3  # Duration for the progress bar in seconds

        try:
            while rclpy.ok():
                # Get images from both sensors
                img_L, [resX_L, resY_L] = self.sim.getVisionSensorImg(
                    self.endscope_L)
                img_R, [resX_R, resY_R] = self.sim.getVisionSensorImg(
                    self.endscope_R)

                # Convert images to NumPy arrays
                img_L = np.frombuffer(
                    img_L, dtype=np.uint8).reshape(resY_L, resX_L, 3)
                img_R = np.frombuffer(
                    img_R, dtype=np.uint8).reshape(resY_R, resX_R, 3)

                # Flip and convert images to OpenCV format
                img_L = cv2.flip(cv2.cvtColor(img_L, cv2.COLOR_BGR2RGB), 0)
                img_R = cv2.flip(cv2.cvtColor(img_R, cv2.COLOR_BGR2RGB), 0)

                # Resize images if needed
                if img_L.shape != img_R.shape:
                    img_R = cv2.resize(img_R, (img_L.shape[1], img_L.shape[0]))

                # Horizontally stack images
                hstack_img = np.hstack((img_L, img_R))

                # Resize to 1920x1080 for display
                hstack_resized = cv2.resize(hstack_img, (1920, 1080))

                # Display progress bar if is_start is False
                if not self.is_start:
                    elapsed_time = time.time() - start_time
                    progress = min(elapsed_time / progress_duration, 1.0)
                    self.draw_progress_bar(hstack_resized, progress, (300, 40))
                    self.draw_progress_bar(hstack_resized, progress, (1260, 40))
                    direction_text = {
                        (0, 0, 0): "Press the Button to Start",
                        (0, -1, 0): "Move towards right",
                        (1, 0, 0): "Move towards front",
                        (0, 0, 1): "Move towards up"
                    }

                    direction_tuple = tuple(self.direction)
                    if direction_tuple in direction_text:
                        cv2.putText(hstack_resized, direction_text[direction_tuple], (200, 1040),
                                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 4, cv2.LINE_AA)
                        cv2.putText(hstack_resized, direction_text[direction_tuple], (1160, 1040),
                                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 4, cv2.LINE_AA)
                # Display the image
                cv2.imshow(self.window_name, hstack_resized)

                # Handle key events
                key = cv2.waitKey(1) & 0xFF
                if key == ord('f'):  # Toggle fullscreen
                    self.fullscreen = not self.fullscreen
                    cv2.setWindowProperty(
                        self.window_name,
                        cv2.WND_PROP_FULLSCREEN,
                        cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL
                    )
                elif key == ord('r'):  # Manually trigger countdown
                    self.is_start = True
                elif key == 27:  # ESC key to exit
                    break

                # If ROS 2 message triggers start, show countdown
                if self.is_start:
                    self.display_countdown()
                    self.is_start = False
                    start_time = time.time()  # Reset progress bar timer

                # Process ROS 2 messages
                rclpy.spin_once(self, timeout_sec=0.01)

                self.sim.step()  # Proceed simulation step
        finally:
            self.sim.stopSimulation()
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    vision_node = VisionSystem()

    # Run ROS 2 and Vision Loop in separate threads
    vision_thread = threading.Thread(target=vision_node.run_vision_loop)
    vision_thread.start()

    rclpy.spin(vision_node)

    vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
