from launch import LaunchDescription
from launch_ros.actions import Node,SetParameter

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2bag_to_csv',            # Package name
            executable='csv_to_json',             # Execution name
            name='csv_to_json_node'
        ),
        SetParameter(name='/ros_version', value='ros2'),
    ])

