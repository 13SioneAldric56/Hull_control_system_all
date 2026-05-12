import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/orangepi/Desktop/Hull_control_ststem/ros2_hull/install/gps_wgs84_to_gcj02'
