from glob import glob
import os

from setuptools import setup

PACKAGE_NAME = "wgs84_gcj_bridge"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    keywords=["ROS2"],
    description="WGS84 to GCJ-02 NavSatFix bridge for map display alignment.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "wgs84_gcj_bridge_node = wgs84_gcj_bridge.bridge_node:main",
        ],
    },
)
