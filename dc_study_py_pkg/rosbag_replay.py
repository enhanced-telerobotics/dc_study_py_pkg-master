import rosbag2_py
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read_rosbag(bag_path, topic_name="/pose_topic"):
    # Initialize rosbag reader
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Get all topics and types
    topic_type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_type = get_message(topic_type_map[topic_name])
    
    # Store position data
    x_data, y_data = [], []

    while reader.has_next():
        (topic, data, _) = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, msg_type)
            position = msg.pose.position
            x_data.append(position.x)
            y_data.append(position.y)

    return x_data, y_data

def plot_overlay(bag1_path, bag2_path, bag3_path, topic_name="/pose_topic"):
    # Read data from both bag files
    x1_data, y1_data = read_rosbag(bag1_path, topic_name)
    x2_data, y2_data = read_rosbag(bag2_path, topic_name)
    x3_data, y3_data = read_rosbag(bag3_path, topic_name)

    # Plot both trajectories
    plt.figure(figsize=(10, 7))
    plt.plot(x1_data, y1_data, marker='.', markersize=0.1, label='no metal Trajectory', color='blue', alpha=0.2)
    plt.plot(x2_data, y2_data, marker='x', markersize=0.1,label='screw metal Trajectory', color='red', alpha=0.5)  # Red line with 0.5 alpha
    plt.plot(x3_data, y3_data, marker='+', markersize=0.1, label='huge metal Trajectory', color='green', alpha=1)  # Red line with 0.5 alpha
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title("Overlayed 2D Trajectories (X-Y) from 3 viper trajectories")
    plt.grid(True)
    plt.legend()

    # Ensure equal scaling
    plt.axis('equal')  # Equal aspect ratio for x and y axes
    plt.show()

def main():
    bag1_path = "/home/erie_lab/Desktop/rosbag2_2025_01_06-16_21_55"  # Change to your first bag directory
    bag2_path = "/home/erie_lab/Desktop/rosbag2_2025_01_06-16_14_50_metal"  # Change to your second bag directory
    bag3_path = "/home/erie_lab/Desktop/rosbag2_2025_01_06-16_30_33_huge_metal"  # Change to your second bag directory
    
    topic_name = "/left_pose"  # Replace with your topic
    plot_overlay(bag1_path, bag2_path, bag3_path, topic_name)

if __name__ == '__main__':
    main()
#Calculated viper_offset: [0.00095297695626284, -0.0002347290550877, -0.0018775880616544]
#//: after breakCalculated viper_offset: [-0.0049981080740409, 0.0042255640659565, 0.0029679589415783]