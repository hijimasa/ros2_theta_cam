from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_theta_cam',
            executable='theta_cam',
            name='theta_cam',
            parameters=[
                {'mode': "'4K'"}, # Image resolution: '4K' or '2K'
                {'serial': "''"}, # Serial number:    example:'001234556'
            ],
            output='screen',
        )
    ])
