import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('sim_diff_drive_bringup')
    shared_param_file = os.path.join(pkg_bringup, 'config', 'green_sphere_following_config.yaml')

    image_flipper_node = Node(
        package='sim_diff_drive_vision',
        executable='image_flipper',
        name='image_flipper'
    )
        
    green_sphere_following_node = Node(
        package='sim_diff_drive_vision',
        executable='green_sphere_following',
        name='green_sphere_following',
        parameters=[shared_param_file]
    )
  
    odometry_localization_node = Node(
        package='sim_diff_drive_control',
        executable='odometry_localization',
        name='odometry_localization',
        parameters=[shared_param_file]
    )

    pid_point_controller_node = Node(
        package='sim_diff_drive_control',
        executable='pid_point_controller',
        name='pid_point_controller',
        parameters=[shared_param_file]  
    )

    return LaunchDescription([
        image_flipper_node,
        green_sphere_following_node,
        odometry_localization_node,
        pid_point_controller_node
    ])