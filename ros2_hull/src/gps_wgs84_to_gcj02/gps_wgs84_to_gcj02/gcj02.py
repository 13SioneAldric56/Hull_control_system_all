"""Approximate WGS84 -> GCJ-02 (China mars) conversion (public algorithm)."""
import math

_A = 6378245.0
_EE = 0.00669342162296594323


def _out_of_china(lng: float, lat: float) -> bool:
    return not (73.66 < lng < 135.05 and 3.86 < lat < 53.55)


def _transform_lat(lng: float, lat: float) -> float:
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * math.pi) + 40.0 * math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320.0 * math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
    return ret


def _transform_lng(lng: float, lat: float) -> float:
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * math.pi) + 40.0 * math.sin(lng / 3.0 * math.pi)) * 2.0 / 3.0
    return ret


def wgs84_to_gcj02(lat_wgs: float, lon_wgs: float) -> tuple:
    """Return (lat_gcj, lon_gcj). Outside China mainland returns input unchanged."""
    if _out_of_china(lon_wgs, lat_wgs):
        return lat_wgs, lon_wgs
    dlat = _transform_lat(lon_wgs - 105.0, lat_wgs - 35.0)
    dlng = _transform_lng(lon_wgs - 105.0, lat_wgs - 35.0)
    rad_lat = lat_wgs / 180.0 * math.pi
    magic = 1.0 - _EE * math.sin(rad_lat) * math.sin(rad_lat)
    sqrt_magic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((_A * (1.0 - _EE)) / (magic * sqrt_magic) * math.pi)
    dlng = (dlng * 180.0) / (_A / sqrt_magic * math.cos(rad_lat) * math.pi)
    return lat_wgs + dlat, lon_wgs + dlng
