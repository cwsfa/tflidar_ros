from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tflidar_ros',
            executable='tflidar_ros_node',
            name='tflidar_ros_node',
            output='screen',
			parameters=[
                {'serial_port': "/dev/ttyUSB0"},
                {'baud_rate': 115200},
                {'model': "TF03"}
			]
        ),
    ])
