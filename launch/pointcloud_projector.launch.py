from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pc2image',
            executable='pointcloud_projector',
            name='pointcloud_projector',
            output='screen',
            parameters=[
                # Add any parameters here if needed
            ],
            remappings=[
                # Add any topic remappings here if needed
                # ('/input_topic', '/your_topic'),
            ],
        ),
    ])
