from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Realsense driver
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            output='screen',
            parameters=[{
                'enable_depth': True,
                'enable_color': True,
                'enable_gyro': False,
                'enable_accel': False,
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': 30,
            }]
        ),

        # Depthimage to laserscan
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            output='screen',
            remappings=[
                ('depth', '/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan')
            ],
            parameters=[{
                'output_frame': 'camera_depth_optical_frame',
                'range_min': 0.2,
                'range_max': 5.0,
                'scan_height': 10,   # average over 10 rows to reduce noise
            }]
        )
    ])