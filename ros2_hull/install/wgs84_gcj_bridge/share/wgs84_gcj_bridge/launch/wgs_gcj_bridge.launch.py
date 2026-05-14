from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        "input_topic",
        default_value="/fix",
        description="NavSatFix input (typically WGS84 from NMEA driver).",
    )
    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/fix_gcj02",
        description="NavSatFix output on GCJ-02 for Mars-datum map tiles.",
    )

    bridge = Node(
        package="wgs84_gcj_bridge",
        executable="wgs84_gcj_bridge_node",
        name="wgs84_gcj_bridge",
        parameters=[
            {
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "output_frame_id": "",
            }
        ],
        output="screen",
    )

    return LaunchDescription([input_topic_arg, output_topic_arg, bridge])
