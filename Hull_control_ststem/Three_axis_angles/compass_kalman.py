"""兼容层：请改用 ``from compass import CompassKalmanFilter``。"""

import warnings

warnings.warn(
    "Three_axis_angles.compass_kalman 已弃用，请使用 compass 包",
    DeprecationWarning,
    stacklevel=2,
)

from compass.filter import (  # noqa: F401
    AdaptiveCompassKalmanFilter,
    CompassKalmanFilter,
    FilteredCompassData,
    KalmanFilter1D,
)

_global_filter = None
_global_compass = None


def reset_filter():
    global _global_filter
    if _global_filter:
        _global_filter.reset()


def close_filter():
    global _global_filter, _global_compass
    if _global_filter:
        _global_filter.reset()
    if _global_compass:
        _global_compass.disconnect()
        _global_compass = None


class ContinuousFilteredReader:
    """兼容：请使用 compass.DDM350B.iter_samples + CompassKalmanFilter。"""

    def __init__(self, port="/dev/ttyS0", process_noise=0.001, measurement_noise=0.1, use_adaptive=False, interval=0.1):
        from compass import CompassConfig, DDM350B, KalmanConfig

        self._compass = DDM350B(
            CompassConfig(
                port=port,
                kalman=KalmanConfig(
                    enabled=True,
                    process_noise=process_noise,
                    measurement_noise=measurement_noise,
                    adaptive=use_adaptive,
                ),
            )
        )
        self._interval = interval

    def __enter__(self):
        self._compass.connect()
        return self

    def __exit__(self, *args):
        self._compass.disconnect()

    def __iter__(self):
        import time

        for sample in self._compass.iter_samples(self._interval):
            from compass.filter import FilteredCompassData

            yield FilteredCompassData(
                sample.roll,
                sample.pitch,
                sample.heading,
                0.0,
                0.0,
                0.0,
            )
