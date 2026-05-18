#!/usr/bin/env python3
"""DDM350B 罗盘 ROS2 节点。"""

from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float64

# 将 Hull_control_ststem 加入路径以导入 compass 包
_WS_ROOT = Path(__file__).resolve().parents[4]
_HULL_PKG = _WS_ROOT / "Hull_control_ststem"
if str(_HULL_PKG) not in sys.path:
    sys.path.insert(0, str(_HULL_PKG))

from compass import CompassConfig, DDM350B, OutputMode  # noqa: E402
from compass.config import Axis, HeadingFormat, KalmanConfig, NorthReference  # noqa: E402


def _mode_from_str(name: str) -> OutputMode:
    key = name.lower().replace("-", "_")
    mapping = {m.name.lower(): m for m in OutputMode}
    if key in mapping:
        return mapping[key]
    aliases = {
        "polling": OutputMode.POLLING,
        "auto_50hz": OutputMode.AUTO_50HZ,
        "auto_50": OutputMode.AUTO_50HZ,
    }
    return aliases.get(key, OutputMode.AUTO_50HZ)


class DdmCompassNode(Node):
    def __init__(self):
        super().__init__("ddm_compass_node")
        self.declare_parameter("port", "/dev/ttyS0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("mode", "AUTO_50HZ")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("kalman", False)
        self.declare_parameter("process_noise", 0.001)
        self.declare_parameter("measurement_noise", 0.1)
        self.declare_parameter("heading_format", "0-360")
        self.declare_parameter("north_reference", "magnetic")
        self.declare_parameter("magnetic_declination_deg", 0.0)

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        mode = _mode_from_str(self.get_parameter("mode").get_parameter_value().string_value)
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        kalman = self.get_parameter("kalman").get_parameter_value().bool_value
        pn = self.get_parameter("process_noise").get_parameter_value().double_value
        mn = self.get_parameter("measurement_noise").get_parameter_value().double_value
        hf = self.get_parameter("heading_format").get_parameter_value().string_value
        nr = self.get_parameter("north_reference").get_parameter_value().string_value
        decl = self.get_parameter("magnetic_declination_deg").get_parameter_value().double_value

        heading_format = (
            HeadingFormat.DEG180 if hf in ("±180", "180", "pm180") else HeadingFormat.DEG360
        )
        north_ref = (
            NorthReference.TRUE if nr.lower() == "true" else NorthReference.MAGNETIC
        )

        self._compass = DDM350B(
            CompassConfig(
                port=port,
                baudrate=baudrate,
                mode=mode,
                axis=Axis.ALL,
                kalman=KalmanConfig(
                    enabled=kalman,
                    process_noise=pn,
                    measurement_noise=mn,
                ),
                heading_format=heading_format,
                north_reference=north_ref,
                magnetic_declination_deg=decl,
                auto_connect=False,
            )
        )
        if not self._compass.connect():
            raise RuntimeError(f"无法连接罗盘 {port}")

        self._pub_heading = self.create_publisher(Float64, "compass/heading", 10)
        self._pub_sample = self.create_publisher(Vector3, "compass/sample", 10)
        period = 1.0 / rate_hz if rate_hz > 0 else 0.02
        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(f"ddm_compass 已启动 port={port} rate={rate_hz}Hz")

    def _on_timer(self):
        sample = self._compass.read_full()
        if sample is None:
            return
        h = Float64()
        h.data = float(sample.heading)
        self._pub_heading.publish(h)
        v = Vector3()
        v.x = float(sample.roll)
        v.y = float(sample.pitch)
        v.z = float(sample.heading)
        self._pub_sample.publish(v)

    def destroy_node(self):
        self._compass.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DdmCompassNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
