from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',  # Replace with your package name
            executable='jetsonMain.py',
            output='screen',
            emulate_tty=True,
            environment={'ROS_DOMAIN_ID': '48'}  # Sets the domain ID for this node
        )
    ])