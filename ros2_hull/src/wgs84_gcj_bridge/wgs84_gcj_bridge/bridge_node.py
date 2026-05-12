"""ROS 2 node: subscribe WGS84 NavSatFix on /fix, publish GCJ-02-aligned fix for map display."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from wgs84_gcj_bridge.gcj02 import wgs84_to_gcj02


class Wgs84GcjBridge(Node):
    def __init__(self) -> None:
        super().__init__("wgs84_gcj_bridge")

        self.declare_parameter("input_topic", "/fix")
        self.declare_parameter("output_topic", "/fix_gcj02")
        self.declare_parameter("output_frame_id", "")

        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._out_frame_param = self.get_parameter("output_frame_id").get_parameter_value().string_value

        self._pub = self.create_publisher(NavSatFix, out_topic, 10)
        self.create_subscription(NavSatFix, in_topic, self._cb, 10)
        self.get_logger().info("WGS84->GCJ02: %s -> %s" % (in_topic, out_topic))

    def _cb(self, msg: NavSatFix) -> None:
        if msg.status.status < NavSatStatus.STATUS_FIX:
            self._pub.publish(msg)
            return

        lat = msg.latitude
        lon = msg.longitude
        if math.isnan(lat) or math.isnan(lon):
            self._pub.publish(msg)
            return

        out = NavSatFix()
        out.header = msg.header
        if self._out_frame_param:
            out.header.frame_id = self._out_frame_param

        glat, glon = wgs84_to_gcj02(lat, lon)
        out.latitude = glat
        out.longitude = glon
        out.altitude = msg.altitude
        out.status = msg.status
        out.position_covariance = msg.position_covariance
        out.position_covariance_type = msg.position_covariance_type
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = Wgs84GcjBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()
