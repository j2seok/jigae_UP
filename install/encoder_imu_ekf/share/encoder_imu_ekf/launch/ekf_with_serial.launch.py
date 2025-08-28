from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='encoder_imu_ekf',
            executable='encoder_imu_ekf',
            name='serial_imu_encoder_node',
            output='screen'
        ),



    ])
