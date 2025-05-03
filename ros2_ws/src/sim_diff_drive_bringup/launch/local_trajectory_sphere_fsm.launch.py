from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    image_flipper_node = Node(
        package='sim_diff_drive_vision',
        executable='image_flipper',
        name='image_flipper',
        output='screen'
    )
        
    sphere_detection_node = Node(
        package='sim_diff_drive_vision',
        executable='sphere_detection',
        name='sphere_detection',
        output='screen'
    )

    odometry_localization_node = Node(
        package='sim_diff_drive_control',
        executable='odometry_localization',
        name='odometry_localization'
    )

    pid_point_controller_node = Node(
        package='sim_diff_drive_control',
        executable='pid_point_controller',
        name='pid_point_controller'
    )

    fsm_node = Node(
        package='sim_diff_drive_behavior',
        executable='local_trajectory_sphere_fsm',
        name='local_trajectory_sphere_fsm',
        output='screen'
    )

    return LaunchDescription([
        image_flipper_node,
        sphere_detection_node,
        odometry_localization_node,
        pid_point_controller_node,
        fsm_node,
    ])