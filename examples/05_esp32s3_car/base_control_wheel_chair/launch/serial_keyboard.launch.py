from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyUSB0", description="Serial port path"
    )
    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate", default_value="115200", description="Serial baudrate"
    )

    teleop_node = Node(
        package="base_control_wheel_chair",
        executable="keyboard_teleop",
        name="keyboard_teleop",
        output="screen",
    )

    return LaunchDescription(
        [
            serial_port_arg,
            baud_rate_arg,
            Node(
                package="base_control_wheel_chair",
                executable="robot_serial",
                name="robot_serial_node",
                output="screen",
                arguments=[
                    LaunchConfiguration("serial_port"),
                    LaunchConfiguration("baud_rate"),
                ],
            ),
            teleop_node,
        ]
    )
