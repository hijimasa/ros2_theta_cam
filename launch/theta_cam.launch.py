from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_theta_cam',
            executable='theta_cam',
            name='theta_cam',
            parameters=[
                {'mode': "'2K'"}, # Image resolution: '4K' or '2K'
                {'serial': "''"}, # Serial number:    example:'001234556'
                {'frame_skip': 1}, # Frame skip: 1=no skip, 3=publish 1 out of 3 frames (30fps->10fps)
            ],
            output='screen',
            respawn=True,
        )
    ])
