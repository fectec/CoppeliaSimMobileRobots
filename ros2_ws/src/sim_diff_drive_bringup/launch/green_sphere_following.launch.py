from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    green_sphere_following_node = Node(
    package='sim_diff_drive_vision',
    executable='green_sphere_following',
    name='green_sphere_following',
    )
  
    localization_node = Node(
        package='sim_diff_drive_control',
        executable='localization',
        name='localization'
    )

    point_controller_node = Node(
        package='sim_diff_drive_control',
        executable='point_controller',
        name='point_controller'
    )

    return LaunchDescription([
        green_sphere_following_node,
        localization_node,
        point_controller_node
    ])