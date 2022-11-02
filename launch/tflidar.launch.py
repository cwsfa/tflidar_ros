from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    model = LaunchConfiguration('model', default="TF03")
    serial_port = LaunchConfiguration('serial_port', default="/dev/ttyUSB0")
    baud_rate = LaunchConfiguration('baud_rate', default=115200)
    topic_name = LaunchConfiguration('topic_name', default="range_feedback")
    frame_link = LaunchConfiguration('frame_link', default="TFlidar")

    return LaunchDescription([
        # declare arguments
        DeclareLaunchArgument('serial_port', default_value=serial_port),
        DeclareLaunchArgument('baud_rate', default_value=baud_rate),
        DeclareLaunchArgument('model', default_value=model),
        DeclareLaunchArgument('topic_name', default_value=topic_name),
        DeclareLaunchArgument('frame_link', default_value=frame_link),
        # run tflidar_ros_node
        Node(
            package='tflidar_ros',
            executable='tflidar_ros_node',
            name='tflidar_ros_node',
            output='screen',
			parameters=[
                {'model': model},
                {'serial_port': serial_port},
                {'baud_rate': baud_rate},
                {'topic_name': topic_name},
                {'frame_link': frame_link},
			]
        ),
    ])
