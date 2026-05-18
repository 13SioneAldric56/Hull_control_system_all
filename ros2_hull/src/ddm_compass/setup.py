from glob import glob
import os

from setuptools import setup

PACKAGE_NAME = "ddm_compass"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    license="MIT",
    entry_points={
        "console_scripts": [
            "ddm_compass_node = ddm_compass.ddm_compass_node:main",
        ],
    },
)
