from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    marker_length = LaunchConfiguration('marker_length_m', default='0.04')
    id_left = LaunchConfiguration('id_left', default='13')
    id_right = LaunchConfiguration('id_right', default='14')
    x_thresh = LaunchConfiguration('x_thresh_cm', default='5.0')
    theta_thresh = LaunchConfiguration('theta_thresh_deg', default='5.0')
    use_depth = LaunchConfiguration('use_depth', default='false')
    target_theta = LaunchConfiguration('target_theta_deg', default='180.0')
    approach_step = LaunchConfiguration('approach_step_cm', default='10.0')
    camera_x_offset_cm = LaunchConfiguration('camera_x_offset_cm')
    x_close_cm         = LaunchConfiguration('x_close_cm')
    theta_close_deg    = LaunchConfiguration('theta_close_deg')
    min_cmd_interval   = LaunchConfiguration('min_cmd_interval')


    return LaunchDescription([
        DeclareLaunchArgument('marker_length_m', default_value='0.04'),
        DeclareLaunchArgument('id_left', default_value='13'),
        DeclareLaunchArgument('id_right', default_value='14'),
        DeclareLaunchArgument('x_thresh_cm', default_value='5.0'),
        DeclareLaunchArgument('theta_thresh_deg', default_value='5.0'),
        DeclareLaunchArgument('use_depth', default_value='false'),
        DeclareLaunchArgument('target_theta_deg', default_value='0.0'),
        DeclareLaunchArgument('approach_step_cm', default_value='10.0'),
        DeclareLaunchArgument('camera_x_offset_cm', default_value='4.0'),
        DeclareLaunchArgument('x_close_cm',        default_value='1.0'),
        DeclareLaunchArgument('theta_close_deg',   default_value='2.0'),
        DeclareLaunchArgument('min_cmd_interval',  default_value='0.5'),

        Node(
            package='pallet_align', executable='aruco_detector_node', output='screen',
            parameters=[{
                'marker_length_m': marker_length,
                'id_left': id_left,
                'id_right': id_right,
                'x_thresh_cm': x_thresh,
                'theta_thresh_deg': theta_thresh,
                'use_depth': use_depth,
                'target_theta_deg': target_theta,
                'approach_step_cm': approach_step,
                'image_topic': '/d435/color/image_raw',
                'camera_info_topic': '/d435/color/camera_info',
                'camera_x_offset_cm': camera_x_offset_cm,
                'x_close_cm':         x_close_cm,
                'theta_close_deg':    theta_close_deg,
                'min_cmd_interval':   min_cmd_interval,

            }]
        ),
        Node(
            package='pallet_align', executable='serial_sender_node', output='screen'
        ),
    ])
