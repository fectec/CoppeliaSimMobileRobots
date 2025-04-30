from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    green_sphere_following_node = Node(
    package='sim_diff_drive_vision',
    executable='green_sphere_following',
    name='green_sphere_following',
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

    return LaunchDescription([
        green_sphere_following_node,
        odometry_localization_node,
        pid_point_controller_node
    ])