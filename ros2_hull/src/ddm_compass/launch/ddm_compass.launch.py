from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ddm_compass",
                executable="ddm_compass_node",
                name="ddm_compass_node",
                output="screen",
                parameters=[
                    {
                        "port": "/dev/ttyS0",
                        "baudrate": 115200,
                        "mode": "AUTO_50HZ",
                        "rate_hz": 50.0,
                        "kalman": False,
                        "heading_format": "0-360",
                        "north_reference": "magnetic",
                    }
                ],
            ),
        ]
    )
