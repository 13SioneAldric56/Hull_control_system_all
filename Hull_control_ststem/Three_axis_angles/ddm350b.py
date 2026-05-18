"""兼容层：请改用 ``from compass import DDM350B, OutputMode, read_compass``。"""

import warnings

warnings.warn(
    "Three_axis_angles.ddm350b 已弃用，请使用 compass 包",
    DeprecationWarning,
    stacklevel=2,
)

from compass import CompassConfig, CompassData, CompassSample, DDM350B, OutputMode
from compass import read_compass as _read_compass_once

_global_compass = None


def read_compass(port: str = "/dev/ttyS0", mode: OutputMode = OutputMode.POLLING):
    return _read_compass_once(port=port, mode=mode)


def close():
    global _global_compass
    if _global_compass:
        _global_compass.disconnect()
        _global_compass = None
