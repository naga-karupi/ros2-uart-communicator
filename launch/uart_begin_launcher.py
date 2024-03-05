from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [   
            Node(
                package='uart_communicator',
                executable='main',
                exec_name='uart_communicator',
                name='uart_communicator',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {"serial_name": "/dev/ttyACM0"}
                ]
            ),
        ])