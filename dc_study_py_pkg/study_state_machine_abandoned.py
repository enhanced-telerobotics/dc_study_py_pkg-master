import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from std_msgs.msg import Float32
import random

class UserStudyController(Node):
    def __init__(self):
        super().__init__('user_study_controller')

        # Create a client for the 'set_parameters' service of the target node
        self.parameter_client = self.create_client(SetParameters, '/pose_delay_node/set_parameters')

        # Wait for the service to be available
        while not self.parameter_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Waiting for parameter service to become available...')

        self.get_logger().info('Parameter service is available. Starting to alternate delay parameters...')

        # List of delay values to randomly choose from
        self.delay_values = [100, 400, 700]  # Delay times in milliseconds

        # Timer to change the delay parameter every 5 seconds
        self.timer = self.create_timer(1.0, self.alternate_delay_param)
        # Subscribe btn msg and pub to coppeliasim
        self.start_btn_sub = self.create_subscription(
            Float32,
            'Joy',
            self.start_btn_cb,
            10
        )
        self.start_btn_pub = self.create_publisher(
            Float32,
            'usr_controller/start_btn',
            10
        )
    # Sub callbacks
    def start_btn_cb(self, msg):
        pass
    def alternate_delay_param(self):
        # Randomly select a delay value
        delay_value = random.choice(self.delay_values)

        # Prepare the request to change the parameter
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name='delay_time', value=delay_value, type_=Parameter.Type.INTEGER).to_parameter_msg()
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

def main(args=None):
    rclpy.init(args=args)
    node = UserStudyController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
