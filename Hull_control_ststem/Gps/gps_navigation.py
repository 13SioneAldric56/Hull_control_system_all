"""
GPS导航匹配算法
基于NMEA协议解析的经纬度数据进行导航计算
"""

import math
import json
import os
from typing import Tuple, List, Dict, Any, Optional
from dataclasses import dataclass


@dataclass
class NavigationResult:
    """导航结果"""
    distance: float          # 相对距离(米)
    bearing: float          # 相对角度(度, 0-360, 正北为0度)
    distance_km: float      # 相对距离(公里)
    east_distance: float    # 东向位移(米, 正东为正)
    north_distance: float   # 北向位移(米, 正北为正)


@dataclass
class Waypoint:
    """航点信息"""
    name: str
    latitude: float
    longitude: float
    description: str = ""


class GPSNavigation:
    """GPS导航匹配算法"""

    # 地球平均半径(米)
    EARTH_RADIUS_M = 6371000
    # 地球扁率相关常数
    EARTH_A = 6378137.0        # 长半轴
    EARTH_B = 6356752.314245   # 短半轴

    def __init__(self, home_lat: float = 0.0, home_lon: float = 0.0):
        """
        初始化导航器

        Args:
            home_lat: 基准点纬度(度)
            home_lon: 基准点经度(度)
        """
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.waypoints: List[Waypoint] = []  # 航点列表

    def set_home(self, latitude: float, longitude: float) -> None:
        """
        设置基准点/目标点

        Args:
            latitude: 纬度(度)
            longitude: 经度(度)
        """
        self.home_lat = latitude
        self.home_lon = longitude

    @staticmethod
    def calculate_distance(lat1: float, lon1: float,
                          lat2: float, lon2: float) -> Tuple[float, float, float]:
        """
        计算两点间的距离和位移分量(使用Haversine公式)

        Args:
            lat1: 起点纬度(度)
            lon1: 起点经度(度)
            lat2: 终点纬度(度)
            lon2: 终点经度(度)

        Returns:
            (总距离(米), 东向位移(米), 北向位移(米))
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)

        # Haversine公式计算弧长距离
        a = (math.sin(delta_lat / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) *
             math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = GPSNavigation.EARTH_RADIUS_M * c

        # 计算局部平面位移分量(ENU坐标系)
        # 北向分量
        north_distance = delta_lat * GPSNavigation.EARTH_RADIUS_M

        # 东向分量(考虑纬度变化的影响)
        east_distance = delta_lon * GPSNavigation.EARTH_RADIUS_M * math.cos(lat1_rad)

        return distance, east_distance, north_distance

    @staticmethod
    def calculate_bearing(lat1: float, lon1: float,
                         lat2: float, lon2: float) -> float:
        """
        计算从起点到终点的方位角(航向角)

        Args:
            lat1: 起点纬度(度)
            lon1: 起点经度(度)
            lat2: 终点纬度(度)
            lon2: 终点经度(度)

        Returns:
            方位角(度, 0-360, 正北为0度, 顺时针增加)
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)

        # 使用方位角公式
        x = math.sin(delta_lon) * math.cos(lat2_rad)
        y = (math.cos(lat1_rad) * math.sin(lat2_rad) -
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))

        bearing = math.atan2(x, y)
        bearing_deg = math.degrees(bearing)

        # 归一化到0-360度
        bearing_deg = (bearing_deg + 360) % 360

        return bearing_deg

    def get_relative_position(self, current_lat: float, current_lon: float,
                            target_lat: float = None, target_lon: float = None
                            ) -> NavigationResult:
        """
        获取当前位置相对于目标点的导航信息

        Args:
            current_lat: 当前纬度(度)
            current_lon: 当前经度(度)
            target_lat: 目标点纬度(度), 默认为基准点
            target_lon: 目标点经度(度), 默认为基准点

        Returns:
            NavigationResult: 包含距离和角度的导航结果
        """
        if target_lat is None:
            target_lat = self.home_lat
        if target_lon is None:
            target_lon = self.home_lon

        distance, east_dist, north_dist = self.calculate_distance(
            current_lat, current_lon, target_lat, target_lon
        )
        bearing = self.calculate_bearing(
            current_lat, current_lon, target_lat, target_lon
        )

        return NavigationResult(
            distance=distance,
            bearing=bearing,
            distance_km=distance / 1000.0,
            east_distance=east_dist,
            north_distance=north_dist
        )

    @staticmethod
    def navigation_between(lat1: float, lon1: float,
                          lat2: float, lon2: float) -> NavigationResult:
        """
        计算两点间的相对位置和角度(静态接口)

        Args:
            lat1: 起点纬度(度)
            lon1: 起点经度(度)
            lat2: 终点纬度(度)
            lon2: 终点经度(度)

        Returns:
            NavigationResult: 相对距离和角度信息
        """
        distance, east_dist, north_dist = GPSNavigation.calculate_distance(
            lat1, lon1, lat2, lon2
        )
        bearing = GPSNavigation.calculate_bearing(lat1, lon1, lat2, lon2)

        return NavigationResult(
            distance=distance,
            bearing=bearing,
            distance_km=distance / 1000.0,
            east_distance=east_dist,
            north_distance=north_dist
        )

    @staticmethod
    def get_cardinal_direction(bearing: float) -> str:
        """
        将角度转换为方位名称

        Args:
            bearing: 方位角(度)

        Returns:
            方位名称(N, NE, E, SE, S, SW, W, NW)
        """
        directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
        index = round(bearing / 45) % 8
        return directions[index]

    # ==================== 航点管理功能 ====================

    def load_waypoints_from_file(self, filepath: str) -> bool:
        """
        ���JSON文件加载航点

        文件格式示例:
        [
            {
                "name": "航点1",
                "latitude": 31.2304,
                "longitude": 121.4737,
                "description": "起始点"
            }
        ]

        Args:
            filepath: JSON文件路径

        Returns:
            True加载成功, False失败
        """
        try:
            if not os.path.exists(filepath):
                print(f"航点文件不存在: {filepath}")
                return False

            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)

            self.waypoints.clear()
            for item in data:
                waypoint = Waypoint(
                    name=item.get('name', '未命名'),
                    latitude=float(item['latitude']),
                    longitude=float(item['longitude']),
                    description=item.get('description', '')
                )
                self.waypoints.append(waypoint)

            print(f"成功加载 {len(self.waypoints)} 个航点")
            return True

        except Exception as e:
            print(f"加载航点失败: {e}")
            return False

    def save_waypoints_to_file(self, filepath: str) -> bool:
        """
        将航点保存到JSON文件

        Args:
            filepath: JSON文件路径

        Returns:
            True保存成功, False失败
        """
        try:
            data = []
            for wp in self.waypoints:
                data.append({
                    'name': wp.name,
                    'latitude': wp.latitude,
                    'longitude': wp.longitude,
                    'description': wp.description
                })

            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)

            print(f"成功保存 {len(self.waypoints)} 个航点到 {filepath}")
            return True

        except Exception as e:
            print(f"保存航点失败: {e}")
            return False

    def add_waypoint(self, name: str, latitude: float, longitude: float,
                    description: str = "") -> None:
        """
        添加航点

        Args:
            name: 航点名称
            latitude: 纬度
            longitude: 经度
            description: 描述信息
        """
        waypoint = Waypoint(name, latitude, longitude, description)
        self.waypoints.append(waypoint)

    def remove_waypoint(self, name: str) -> bool:
        """
        删除指定名称的航点

        Args:
            name: 航点名称

        Returns:
            True删除成功, False航点不存在
        """
        for i, wp in enumerate(self.waypoints):
            if wp.name == name:
                del self.waypoints[i]
                return True
        return False

    def get_waypoint(self, name: str) -> Optional[Waypoint]:
        """
        获取指定航点

        Args:
            name: 航点名称

        Returns:
            Waypoint对象或None
        """
        for wp in self.waypoints:
            if wp.name == name:
                return wp
        return None

    def list_waypoints(self) -> List[Dict[str, Any]]:
        """
        列出所有航点

        Returns:
            航点信息字典列表
        """
        result = []
        for wp in self.waypoints:
            result.append({
                'name': wp.name,
                'latitude': wp.latitude,
                'longitude': wp.longitude,
                'description': wp.description
            })
        return result

    def find_nearest_waypoint(self, current_lat: float, current_lon: float
                            ) -> Optional[Tuple[Waypoint, NavigationResult]]:
        """
        查找距离当前位置最近的航点

        Args:
            current_lat: 当前纬度
            current_lon: 当前经度

        Returns:
            (最近航点, 导航结果) 或 None
        """
        if not self.waypoints:
            return None

        nearest = None
        min_distance = float('inf')
        nearest_result = None

        for wp in self.waypoints:
            result = self.navigation_between(current_lat, current_lon,
                                             wp.latitude, wp.longitude)
            if result.distance < min_distance:
                min_distance = result.distance
                nearest = wp
                nearest_result = result

        return (nearest, nearest_result) if nearest else None

    def match_nearest_waypoint(self, current_lat: float, current_lon: float,
                              max_distance: float = 5000.0
                              ) -> Optional[Tuple[Waypoint, NavigationResult]]:
        """
        匹配距离当前位置指定范围内的最近航点

        Args:
            current_lat: 当前纬度
            current_lon: 当前经度
            max_distance: 最大匹配距离(米)

        Returns:
            (匹配的航点, 导航结果) 或 None(范围内无航点)
        """
        result = self.find_nearest_waypoint(current_lat, current_lon)
        if result and result[1].distance <= max_distance:
            return result
        return None

    def calculate_all_waypoints(self, current_lat: float, current_lon: float
                               ) -> List[Dict[str, Any]]:
        """
        计算当前位置到所有航点的导航信息

        Args:
            current_lat: 当前纬度
            current_lon: 当前经度

        Returns:
            包含导航结果的航点信息列表，按距离排序
        """
        results = []
        for wp in self.waypoints:
            nav_result = self.navigation_between(
                current_lat, current_lon,
                wp.latitude, wp.longitude
            )
            results.append({
                'name': wp.name,
                'latitude': wp.latitude,
                'longitude': wp.longitude,
                'description': wp.description,
                'distance': nav_result.distance,
                'bearing': nav_result.bearing,
                'distance_km': nav_result.distance_km,
                'direction': self.get_cardinal_direction(nav_result.bearing),
                'east_distance': nav_result.east_distance,
                'north_distance': nav_result.north_distance
            })

        # 按距离排序
        results.sort(key=lambda x: x['distance'])
        return results

    def get_navigation_to_waypoint(self, current_lat: float, current_lon: float,
                                   waypoint_name: str) -> Optional[NavigationResult]:
        """
        计算到指定航点的导航信息

        Args:
            current_lat: 当前纬度
            current_lon: 当前经度
            waypoint_name: 航点名称

        Returns:
            NavigationResult 或 None(航点不存在)
        """
        wp = self.get_waypoint(waypoint_name)
        if wp:
            return self.navigation_between(
                current_lat, current_lon,
                wp.latitude, wp.longitude
            )
        return None


def create_navigation_target(name: str, latitude: float, longitude: float
                            ) -> Dict[str, Any]:
    """
    创建导航目标点

    Args:
        name: 目标点名称
        latitude: 纬度(度)
        longitude: 经度(度)

    Returns:
        目标点信息字典
    """
    return {
        'name': name,
        'latitude': latitude,
        'longitude': longitude
    }


def main():
    """测试和演示"""
    print("=" * 60)
    print("GPS导航匹配算法测试")
    print("=" * 60)

    # 创建导航器
    nav = GPSNavigation()

    # 1. 测试静态接口
    print("\n1. 静态接口测试:")
    result = GPSNavigation.navigation_between(
        31.2304, 121.4737,   # 起点
        31.2400, 121.4800    # 终点
    )
    print(f"  起点到终点: {result.distance:.2f}米, {result.bearing:.1f}°")

    # 2. 测试航点管理
    print("\n2. 航点管理测试:")
    nav.add_waypoint("码头A", 31.2304, 121.4737, "起始码头")
    nav.add_waypoint("港口B", 31.2400, 121.4800, "目标港口")
    nav.add_waypoint("锚地C", 31.2350, 121.4650, "临时锚地")

    print("   已添加航点:")
    for wp in nav.list_waypoints():
        print(f"   - {wp['name']}: ({wp['latitude']:.6f}°, {wp['longitude']:.6f}°)")

    # 3. 测试当前位置计算
    print("\n3. 当前位置导航计算:")
    current_lat, current_lon = 31.2280, 121.4700
    print(f"   当前位置: ({current_lat:.6f}°, {current_lon:.6f}°)")

    results = nav.calculate_all_waypoints(current_lat, current_lon)
    print("   到各航点的距离和方向:")
    for r in results:
        print(f"   {r['name']}: {r['distance']:.1f}米, {r['bearing']:.1f}°({r['direction']})")

    # 4. 测试最近航点匹配
    print("\n4. 最近航点匹配:")
    nearest = nav.find_nearest_waypoint(current_lat, current_lon)
    if nearest:
        wp, res = nearest
        print(f"   最近航点: {wp.name}")
        print(f"   距离: {res.distance:.1f}米")
        print(f"   方位: {res.bearing:.1f}°")

    # 5. 测试航点保存/加载
    print("\n5. 航点文件操作:")
    test_file = "test_waypoints.json"
    nav.save_waypoints_to_file(test_file)

    nav2 = GPSNavigation()
    nav2.load_waypoints_from_file(test_file)
    print(f"   加载了 {len(nav2.waypoints)} 个航点")

    # 清理测试文件
    if os.path.exists(test_file):
        os.remove(test_file)

    # 6. 测试航点匹配功能
    print("\n6. 航点匹配测试 (最大匹配距离5000米):")
    match = nav.match_nearest_waypoint(current_lat, current_lon, max_distance=5000)
    if match:
        wp, res = match
        print(f"   匹配到航点: {wp.name}")
        print(f"   距离: {res.distance:.1f}米")
    else:
        print("   5000米范围内无航点")


if __name__ == '__main__':
    main()
