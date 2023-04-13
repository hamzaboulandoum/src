from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'driver_serial_port', default_value='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0',
            description='Arduino serial port for the driver node'
        ),
        Node(
            package='assistant',
            executable='dirver',
            name='driver',
            parameters=[
                {'serial_port': LaunchConfiguration('driver_serial_port')}
            ]
        ),
        DeclareLaunchArgument(
            'broadcaster_serial_port', default_value='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0',
            description='Arduino serial port for the broadcaster node'
        ),
        Node(
            package='assistant',
            executable='broadcaster',
            name='broadcaster',
            parameters=[
                {'serial_port': LaunchConfiguration('broadcaster_serial_port')}
            ]
        ),
        DeclareLaunchArgument(
            'rplidar_serial_port', default_value='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0',
            description='Serial port for the rplidar node'
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            parameters=[
                {'serial_port': LaunchConfiguration('rplidar_serial_port'),
                "frame_id":"lidar_link",
                "angle_compensate":True,
                "scan_mode":"Standard"}
            ]
        )
    ])