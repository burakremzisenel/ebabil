from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety_stop',
            executable='safety_stop',
            name='safety_stop',
            output='screen',
            parameters=[{
                'topic_depth': '/cam_front/camera/depth/image_rect_raw',
                'topic_info':  '/cam_front/camera/depth/camera_info',
                'stop_distance_m': 1.0,
                'roi_ratio': 0.25,
            }]
        )
    ])
