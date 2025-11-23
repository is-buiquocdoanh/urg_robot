from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Tham số launch để dễ thay đổi khi chạy
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Serial device for Hokuyo URG (dùng /dev/serial/by-id/... cho ổn định)'
    )
    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud', default_value='115200',
        description='Baudrate cho URG-04LX (mặc định 115200)'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='laser',
        description='TF frame id cho laser'
    )

    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        serial_port_arg,
        serial_baud_arg,
        frame_id_arg,

        # Hokuyo URG driver node
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'serial_baud': serial_baud,
                'frame_id': frame_id
            }]
        ),
    ])

