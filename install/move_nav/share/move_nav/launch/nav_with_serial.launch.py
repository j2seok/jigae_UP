from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='move_nav',
            executable='encoder_imu_ekf',
            name='serial_imu_encoder_node',
            output='screen'
        ),



    ])