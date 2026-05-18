"""兼容层：请改用 ``from compass.wrap import HeadingWrapReader``。"""

import warnings

warnings.warn(
    "heading_wrap 模块已弃用，请使用 compass.wrap",
    DeprecationWarning,
    stacklevel=2,
)

from compass.heading import angle_difference, normalize_deg360 as normalize_angle
from compass.wrap import HeadingWrap, HeadingWrapReader, WrappedHeadingData

__all__ = [
    "HeadingWrap",
    "HeadingWrapReader",
    "WrappedHeadingData",
    "angle_difference",
    "normalize_angle",
]
