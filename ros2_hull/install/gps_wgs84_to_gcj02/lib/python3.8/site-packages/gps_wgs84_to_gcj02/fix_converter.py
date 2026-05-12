import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from gps_wgs84_to_gcj02.gcj02 import wgs84_to_gcj02


class FixConverter(Node):
    def __init__(self):
        super().__init__('wgs84_gcj02_converter')
        self.declare_parameter('input_topic', '/fix')
        self.declare_parameter('output_topic', '/fix_gcj')
        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._pub = self.create_publisher(NavSatFix, out_topic, 10)
        self._sub = self.create_subscription(NavSatFix, in_topic, self._cb, 10)
        self.get_logger().info(
            'Subscribing %s, publishing %s (WGS84 -> GCJ-02)' % (in_topic, out_topic))

    def _cb(self, msg: NavSatFix):
        lat_g, lon_g = wgs84_to_gcj02(msg.latitude, msg.longitude)
        out = NavSatFix()
        out.header = msg.header
        out.status = msg.status
        out.latitude = lat_g
        out.longitude = lon_g
        out.altitude = msg.altitude
        out.position_covariance = msg.position_covariance
        out.position_covariance_type = msg.position_covariance_type
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FixConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
