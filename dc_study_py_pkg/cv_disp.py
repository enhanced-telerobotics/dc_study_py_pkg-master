from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
import numpy as np

# Connect to CoppeliaSim
client = RemoteAPIClient()
sim = client.require('sim')

sim.setStepping(True)

# Get handles for both vision sensors
endscope_L = sim.getObject('/Vision_sensor_left')
endscope_R = sim.getObject('/Vision_sensor_right')
robot=sim.getObject('/ee_offset_PSM1')
home = [-1.475, 0, 0.75, 0, np.sqrt(0.5), 0, np.sqrt(0.5)]
# Setup OpenCV window
window_name = 'HStack Vision Sensors'
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

fullscreen = False  # Variable to track fullscreen state

sim.startSimulation()

try:
    while True:
        # Get images from both sensors
        img_L, [resX_L, resY_L] = sim.getVisionSensorImg(endscope_L)
        img_R, [resX_R, resY_R] = sim.getVisionSensorImg(endscope_R)

        # Convert images to NumPy arrays
        img_L = np.frombuffer(img_L, dtype=np.uint8).reshape(resY_L, resX_L, 3)
        img_R = np.frombuffer(img_R, dtype=np.uint8).reshape(resY_R, resX_R, 3)

        # Flip and convert images to OpenCV format
        img_L = cv2.flip(cv2.cvtColor(img_L, cv2.COLOR_BGR2RGB), 0)
        img_R = cv2.flip(cv2.cvtColor(img_R, cv2.COLOR_BGR2RGB), 0)

        # Resize images to ensure consistent dimensions if needed
        if img_L.shape != img_R.shape:
            img_R = cv2.resize(img_R, (img_L.shape[1], img_L.shape[0]))

        # Horizontally stack the images
        hstack_img = np.hstack((img_L, img_R))

        # Resize the stacked image to 1920x1080
        hstack_resized = cv2.resize(hstack_img, (1920, 1080))

        # Display the resized image
        cv2.imshow(window_name, hstack_resized)

        # Handle key events
        key = cv2.waitKey(1) & 0xFF
        if key == ord('f'):  # Toggle fullscreen mode
            fullscreen = not fullscreen
            cv2.setWindowProperty(
                window_name,
                cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL
            )
        if key == ord('r'):
            sim.setObjectPose(robot, home, -1)
        elif key == 27:  # ESC key to exit
            break

        sim.step()  # triggers next simulation step
finally:
    sim.stopSimulation()
    cv2.destroyAllWindows()
