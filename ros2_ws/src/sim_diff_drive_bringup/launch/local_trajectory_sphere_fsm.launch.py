from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sphere_detection_node = Node(
        package='sim_diff_drive_vision',
        executable='sphere_detection',
        name='sphere_detection',
        output='screen'
    )

    fsm_node = Node(
        package='sim_diff_drive_behavior',
        executable='local_trajectory_sphere_fsm',
        name='local_trajectory_sphere_fsm',
        output='screen'
    )

    return LaunchDescription([
        sphere_detection_node,
        fsm_node,
    ])
