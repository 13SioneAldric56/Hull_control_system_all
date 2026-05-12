from setuptools import setup

package_name = 'gps_wgs84_to_gcj02'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='WGS84 to GCJ-02 NavSatFix converter.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'wgs84_gcj02_converter = gps_wgs84_to_gcj02.fix_converter:main',
        ],
    },
)
