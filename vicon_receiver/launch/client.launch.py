from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(name="host"),
        DeclareLaunchArgument(name="buffer_size", default_value='200'),
        DeclareLaunchArgument(name="namespace", default_value='vicon'),
        Node(
            package='vicon_receiver', executable='vicon_client', output='screen',
            parameters=[{'hostname': LaunchConfiguration('host'), 
                         'buffer_size': LaunchConfiguration('buffer_size'), 
                         'namespace': LaunchConfiguration('namespace')
                         }]
        )
    ])
