from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_subscriber_node = Node(
        package='vision',
        executable='camera_subscriber',
        name='camera_subscriber'
    )

    localization_node = Node(
        package='control',
        executable='localization',
        name='localization'
    )

    position_controller_node = Node(
        package='control',
        executable='position_controller',
        name='position_controller'
    )

    return LaunchDescription([
        camera_subscriber_node,
        localization_node,
        position_controller_node
    ])