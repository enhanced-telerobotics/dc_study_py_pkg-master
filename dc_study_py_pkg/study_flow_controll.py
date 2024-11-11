import rclpy
import random
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from geometry_msgs.msg import Vector3, Pose
from sensor_msgs.msg import Joy
import time
import asyncio
import numpy as np
import json
import csv


class SimStudyController(Node):
    def __init__(self, robot):
        super().__init__('sim_usr_subscriber')
        # Json file to story current progress
        self.state_file = '/home/erie_lab/ros2_ws/src/dc_study_py_pkg-master/user_study_state.json'
        self.current_state = 'practice'
        self.study_state = ['practice', 'baseline',
                            'training_task', 'evaluation']
        self.trials_completed = 0
        self.blocks_completed = 0
        self.current_conditions = {
            'delay': None,
            'distance': None,
            'direction': None
        }
        # The button to controll user study go
        self.btn_sub = self.create_subscription(Joy, 'joy', self.btn_cb, 10)
        self.btn_curr_state = False
        self.btn_last_state = False
        # Counter flag
        self.counter_end = False

        # Publishers to control sim behavior
        self.robot_control = self.create_publisher(Vector3, 'robot_cmd', 10)
        self.target_control = self.create_publisher(Vector3, 'target_cmd', 10)
        # self.state_timer = self.create_timer(0.1, self.robot_cmd_cb)
        self.is_home = False
        self.is_move = False
        self.trial_ready = False
        self.trial_end = False
        self.conditions_json = '/home/erie_lab/ros2_ws/src/dc_study_py_pkg-master/trial_conditions.json'
        # Create a client for the 'set_parameters' service of the delay node
        self.parameter_client = self.create_client(
            SetParameters, '/pose_delay_node/set_parameters')
        # Data recorder
        self.is_recording = False
        self.pose_data = self.create_subscription(
            Pose, 'delayed_pose', self.pose_data_cb, 10)
        self.data_csv = '/home/erie_lab/ros2_ws/src/dc_study_py_pkg-master/trial_pose_data.csv'
        self.pose_data_buffer = []
        # Wait for the service to be available
        while not self.parameter_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Waiting for parameter service to become available...')

        self.restore_state()
    # Data recorder

    def pose_data_cb(self, msg):
        # Callback to collect pose data during the trial.
        if self.is_recording:
            timestamp = self.get_clock().now().to_msg().sec + \
                self.get_clock().now().to_msg().nanosec * 1e-9
            self.pose_data_buffer.append([
                timestamp,
                self.current_state,  # Add current state
                self.trials_completed + 1,  # Current trial number
                self.current_conditions['delay'],  # Delay condition
                self.current_conditions['distance'],  # Distance condition
                self.current_conditions['direction'],  # Direction condition
                msg.position.x, msg.position.y, msg.position.z,
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            ])

    async def save_to_csv(self):
        try:
            with open(self.data_csv, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                if csvfile.tell() == 0:
                    csv_writer.writerow([
                        'Timestamp', 'State', 'Trial Number', 'Delay', 'Distance', 'Direction',
                        'P_x', 'P_y', 'P_z', 'O_x', 'O_y', 'O_z', 'O_w'
                    ])

                while self.is_recording:
                    # Write accumulated data to CSV and clear buffer
                    if self.pose_data_buffer:
                        csv_writer.writerows(self.pose_data_buffer)
                        self.pose_data_buffer.clear()

                    # Wait for a small interval before checking the buffer again
                    await asyncio.sleep(0.1)
        except IOError as e:
            self.get_logger().error(f"Failed to write pose data to CSV: {e}")
    # Dynamic adjust Delay

    def alternate_delay_param(self, delay_value):

        # Prepare the request to change the parameter
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name='delay_time', value=delay_value,
                      type_=Parameter.Type.INTEGER).to_parameter_msg()
        ]

        # Call the service asynchronously and add a callback to handle the response
        future = self.parameter_client.call_async(request)
        future.add_done_callback(self.parameter_update_callback)

    def parameter_update_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info("Successfully updated 'delay_time'.")
            else:
                self.get_logger().error("Failed to update 'delay_time'.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
    # Study flow (asynchronized)

    async def run_study(self):
        for state in self.study_state:
            if self.study_state.index(state) < self.study_state.index(self.current_state):
                continue
            self.current_state = state
            self.log_state()  # Save state at the beginning of each phase
            print(f"Starting {state} phase...")
            await self.run_state(state)
            print(f"Completed {state} phase.")

    async def run_state(self, state):
        trial_limit = {'practice': 5, 'baseline': 27,
                       'training_task': 225}.get(state, 0)

        if state == 'training_task':
            for curr_trial_index in range(self.trials_completed, trial_limit):
                await self.run_trial_block(state, curr_trial_index + 1)
        else:
            for curr_trial_index in range(self.trials_completed, trial_limit):
                await self.run_trial_single(state, curr_trial_index + 1)

    async def run_trial_single(self, state, trial_num):
        # Wait until trial_ready is True
        while not self.trial_ready:
            await asyncio.sleep(0.1)  # Asynchronous wait

        trial_conditions = self.get_trial_conditions(
            self.conditions_json, trial_num, state)
        if trial_conditions:
            delay = trial_conditions['delay']
            distance = trial_conditions['distance']
            direction = trial_conditions['direction']

            print(
                f"Starting trial {trial_num} in {state} phase with delay: {delay}, distance: {distance}, direction: {direction}")
            self.home()
            while not self.trial_ready:
                await asyncio.sleep(0.1)  # Asynchronous wait
            self.generate_target(distance, direction)
            self.count_down()
            print("Pleas go to target")
            self.allow_user_go()
            self.counter_end = False

            self.log_state()
            while not self.trial_end:
                await asyncio.sleep(0.1)
            print(f"Trial {trial_num} complete.")
            self.trials_completed = trial_num
            self.is_recording = False
            self.trial_ready = False  # Reset the flag after the trial is done
            self.trial_end = False

    async def run_trial_block(self, state, trial_num):
        while not self.trial_ready:
            await asyncio.sleep(0.1)  # Asynchronous wait

        trial_conditions = self.get_trial_conditions(
            self.conditions_json, trial_num, state)
        if trial_conditions:
            delay = trial_conditions['delay']
            distance = trial_conditions['distance']
            direction = trial_conditions['direction']

            print(
                f"Starting trial {trial_num} in {state} phase with delay: {delay}, distance: {distance}, direction: {direction}")
            self.home()
            while not self.trial_ready:
                await asyncio.sleep(0.1)  # Asynchronous wait
            self.generate_target(distance, direction)
            self.alternate_delay_param(delay)
            self.count_down()
            print("Pleas go to target")
            self.allow_user_go()
            self.counter_end = False

            self.log_state()
            while not self.trial_end:
                await asyncio.sleep(0.1)
            print(f"Trial {trial_num} complete.")
            self.trials_completed = trial_num
            self.blocks_completed = trial_num // 5
            if self.blocks_completed == 10:
                self.test_break()
            self.is_recording = False
            self.trial_ready = False  # Reset the flag after the trial is done
            self.trial_end = False

    def get_trial_conditions(self, file_path, trial_num, state):
        with open(file_path, 'r') as json_file:
            conditions = json.load(json_file)

        # if (state == 'baseline' or state == 'practice') and 0 < trial_num <= len(conditions['baseline']):
        #     return conditions['baseline'][trial_num - 1]
        # elif state == 'training_task' and 0 < trial_num <= len(conditions['training_task']):
        #     return conditions['training_task'][trial_num - 1]
        # else:
        #     print(f"Trial number {trial_num} not found in {state} phase.")
        #     return None

        if 0 < trial_num <= len(conditions.get(state, [])):
            self.current_conditions = conditions[state][trial_num - 1]
            return conditions[state][trial_num - 1]
        else:
            print(f"Trial number {trial_num} not found in {state} phase.")
            return None
    def btn_cb(self, msg):
        if msg.buttons is not None:
            self.btn_curr_state = msg.buttons[3] == 1
            if self.btn_curr_state and not self.btn_last_state:
                self.get_logger().info("Button pressed! Please hold till you finished this run..")
                self.trial_ready = True
            elif not self.btn_curr_state and self.btn_last_state:
                self.trial_end = True
            self.btn_last_state = self.btn_curr_state

    def count_down(self):
        if not self.counter_end:
            print("Starting countdown:")
            for i in range(3, 0, -1):
                print(str(i) + "...")
                time.sleep(1)
            print("Countdown complete!")
            self.counter_end = True

    def allow_user_go(self):
        msg = Vector3()
        msg.x = 0.0
        msg.y = 1.0
        self.robot_control.publish(msg)
        self.is_recording = True
        print("Pose recording started.")
        asyncio.create_task(self.save_to_csv())

    def test_break(self):
        print('Time for break')

    def home(self):
        msg = Vector3()
        msg.x = 1.0
        msg.y = 0.0
        self.robot_control.publish(msg)

    def generate_target(self, distance, direction):
        # Ensure the home position is defined
        # Replace with `self.home` if defined elsewhere
        home_pos = np.array([-1.475, 0, 0.75])

        # Convert the `direction` parameter to a unit vector
        if direction == 'up':
            dir_vector = np.array([0, 0, 1])  # Up direction
        elif direction == 'diag':
            # Diagonal direction (normalized)
            dir_vector = np.array([1, 1, 1]) / np.sqrt(3)
        elif direction == 'right':
            dir_vector = np.array([0, -1, 0])  # Down direction
        else:
            raise ValueError(f"Unknown direction: {direction}")

        # Calculate the target position by scaling the direction vector with `distance`
        target_pos = (home_pos + (dir_vector * distance))
        print(f'Sending target position at {target_pos}')

        msg = Vector3()
        msg.x = target_pos[0]
        msg.y = target_pos[1]
        msg.z = target_pos[2]

        self.target_control.publish(msg)

    def log_state(self):
        state_data = {
            'current_state': self.current_state,
            'trials_completed': self.trials_completed,
            'blocks_completed': self.blocks_completed
        }

        try:
            with open(self.state_file, 'w') as json_file:
                json.dump(state_data, json_file)
            print("State logged successfully.")
        except IOError as e:
            self.get_logger().error(f"Failed to log state: {e}")

    def restore_state(self):
        try:
            with open(self.state_file, 'r') as json_file:
                state_data = json.load(json_file)
                self.current_state = state_data.get(
                    'current_state', 'practice')
                self.trials_completed = state_data.get('trials_completed', 0)
                self.blocks_completed = state_data.get('blocks_completed', 0)
                print(
                    f"Restored state: {self.current_state}, Trials Completed: {self.trials_completed}, Blocks Completed: {self.blocks_completed}")
        except (IOError, json.JSONDecodeError):
            self.get_logger().warning("No previous state found. Starting from the beginning.")


def main(args=None):
    rclpy.init(args=args)
    node = SimStudyController(0)

    loop = asyncio.get_event_loop()
    try:
        loop.create_task(node.run_study())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            loop.run_until_complete(asyncio.sleep(0.01))
    finally:
        rclpy.shutdown()
        loop.close()


if __name__ == '__main__':
    main()
