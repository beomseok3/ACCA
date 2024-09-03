from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'erp_port',
            default_value='/dev/ttyUSB2',
            description='Serial port for ERP42'
        ),

        Node(
            package='erp42_communication',
            executable='erp42_serial',
            name='erp42_serial',
            output='screen',
            parameters=[{
                'erp_port': LaunchConfiguration('erp_port')
            }]
        ),
    ])
