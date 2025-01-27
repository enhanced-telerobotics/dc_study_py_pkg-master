import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='delcomp_cpp_pkg',
            executable='abs_teleop',
            name='abs_teleop',
            output='screen'
        ),
        # Node(
        #     package='delcomp_cpp_pkg',
        #     executable='delay_node',
        #     name='delay_node',
        #     output='screen'
        # ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
    ])
