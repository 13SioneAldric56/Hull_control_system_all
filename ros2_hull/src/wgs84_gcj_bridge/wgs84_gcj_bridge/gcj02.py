"""WGS84 to GCJ-02 geodetic transform (degrees). Uses standard mainland-China datum correction."""

import math
from typing import Tuple

_PI = math.pi
_A = 6378245.0
_EE = 0.00669342162296594323


def out_of_china(lat_deg: float, lon_deg: float) -> bool:
    """Rough boundary: GCJ correction not applied outside this box."""

    return not (72.004 <= lon_deg <= 137.8347 and 0.8293 <= lat_deg <= 55.8271)


def _lat_offset(lon_m: float, lat_m: float) -> float:
    ret = -100.0 + 2.0 * lon_m + 3.0 * lat_m + 0.2 * lat_m * lat_m + 0.1 * lon_m * lat_m + 0.2 * math.sqrt(abs(lon_m))
    ret += (20.0 * math.sin(6.0 * lon_m * _PI) + 20.0 * math.sin(2.0 * lon_m * _PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat_m * _PI) + 40.0 * math.sin(lat_m / 3.0 * _PI)) * 2.0 / 3.0
    ret += (
        (160.0 * math.sin(lat_m / 12.0 * _PI) + 320.0 * math.sin(lat_m * _PI / 30.0)) * 2.0 / 3.0
    )
    return ret


def _lon_offset(lon_m: float, lat_m: float) -> float:
    ret = 300.0 + lon_m + 2.0 * lat_m + 0.1 * lon_m * lon_m + 0.1 * lon_m * lat_m + 0.1 * math.sqrt(abs(lon_m))
    ret += (20.0 * math.sin(6.0 * lon_m * _PI) + 20.0 * math.sin(2.0 * lon_m * _PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lon_m * _PI) + 40.0 * math.sin(lon_m / 3.0 * _PI)) * 2.0 / 3.0
    ret += (
        (150.0 * math.sin(lon_m / 12.0 * _PI) + 300.0 * math.sin(lon_m / 30.0 * _PI)) * 2.0 / 3.0
    )
    return ret


def wgs84_to_gcj02(lat_deg: float, lon_deg: float) -> Tuple[float, float]:
    """Degrees WGS84 -> degrees GCJ-02. Outside the China approximation box, inputs are returned unchanged."""

    if out_of_china(lat_deg, lon_deg):
        return lat_deg, lon_deg
    lon_m = lon_deg - 105.0
    lat_m = lat_deg - 35.0
    d_lat = _lat_offset(lon_m, lat_m)
    d_lon = _lon_offset(lon_m, lat_m)
    rad_lat = lat_deg / 180.0 * _PI
    magic = math.sin(rad_lat)
    magic = 1.0 - _EE * magic * magic
    sqrt_magic = math.sqrt(magic)
    d_lat = (d_lat * 180.0) / ((_A * (1.0 - _EE)) / (magic * sqrt_magic) * _PI)
    d_lon = (d_lon * 180.0) / (_A / sqrt_magic * math.cos(rad_lat) * _PI)
    return lat_deg + d_lat, lon_deg + d_lon
