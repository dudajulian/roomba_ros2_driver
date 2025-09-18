from launch import LaunchDescription
from launch_ros.actions import Node

# TODO check why parameters are not loaded from yaml file
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roomba_ros2_driver',
            executable='roomba_ros2_driver',
            name='roomba_ros2_driver',
            output='screen',
            parameters=['config/roomba_driver_config.yaml']
        )
    ])
