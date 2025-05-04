import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('sim_diff_drive_bringup')
    shared_param_file = os.path.join(pkg_bringup, 'config', 'local_trajectory_sphere_fsm_config.yaml')

    image_flipper_node = Node(
        package='sim_diff_drive_vision',
        executable='image_flipper',
        name='image_flipper'
    )
        
    sphere_detection_node = Node(
        package='sim_diff_drive_vision',
        executable='sphere_detection',
        name='sphere_detection',
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

    local_trajectory_sphere_fsm_node = Node(
        package='sim_diff_drive_behavior',
        executable='local_trajectory_sphere_fsm',
        name='local_trajectory_sphere_fsm',
        parameters=[shared_param_file]
    )

    return LaunchDescription([
        image_flipper_node,
        sphere_detection_node,
        odometry_localization_node,
        pid_point_controller_node,
        local_trajectory_sphere_fsm_node,
    ])