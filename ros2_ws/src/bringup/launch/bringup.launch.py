from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    localization_node = Node(
        package='control',
        executable='localization',
        name='localization'
    )

    point_controller_node = Node(
        package='control',
        executable='point_controller',
        name='point_controller'
    )

    visual_servoing_node = Node(
        package='vision',
        executable='visual_servoing',
        name='visual_servoing'
    )

    return LaunchDescription([
        visual_servoing_node,
        localization_node,
        point_controller_node
    ])