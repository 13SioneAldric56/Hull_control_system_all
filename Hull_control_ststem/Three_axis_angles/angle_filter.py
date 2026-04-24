"""
三轴角度数据处理模块 - 低通滤波消抖

功能：
- 航向角 Yaw: 环形角度处理 (-180°~+180°)，消除 0°/360° 跳变
- 俯仰 Pitch、横滚 Roll: 线性角度限幅归一化
- 一阶低通滤波消抖，滤波系数可调节
- 输出稳定、平滑、无跳变的角度

使用示例:
    from angle_filter import AngleFilter

    # 创建滤波器（可单独或同时使用）
    yaw_filter = AngleFilter(filter_coeff=0.15)      # Yaw 环形角度滤波
    pitch_filter = AngleFilter(filter_coeff=0.15)    # Pitch 线性角度滤波
    roll_filter = AngleFilter(filter_coeff=0.15)    # Roll 线性角度滤波

    # 处理角度
    yaw_out = process_yaw(raw_yaw)      # Yaw 滤波后转回 0~360°
    pitch_out = process_pitch(raw_pitch)  # Pitch 线性角度
    roll_out = process_roll(raw_roll)      # Roll 线性角度
"""

from typing import Tuple, NamedTuple
from dataclasses import dataclass, field


@dataclass
class AngleFilter:
    """
    一阶低通滤波器 - 专用于角度数据处理

    Attributes:
        filter_coeff: 滤波系数 (0.0~1.0)
            - 越小越���滑（响应慢）
            - 越大响应越快（噪声大）
            - 推荐值: 0.1~0.3
        initial_value: 滤波器初始值
    """

    filter_coeff: float = 0.15
    _last_value: float = field(default=float('nan'), init=False, repr=False)
    _initialized: bool = field(default=False, init=False, repr=False)

    def __post_init__(self):
        self.filter_coeff = max(0.0, min(1.0, self.filter_coeff))

    def filter(self, raw_value: float) -> float:
        """
        执行一阶低通滤波

        Args:
            raw_value: 原始角度值

        Returns:
            滤波后的角度值
        """
        if not self._initialized:
            self._last_value = raw_value
            self._initialized = True
            return raw_value

        filtered = self.filter_coeff * raw_value + (1 - self.filter_coeff) * self._last_value
        self._last_value = filtered
        return filtered

    def reset(self, value: float = None):
        """重置滤波器状态"""
        self._initialized = False
        if value is not None:
            self._last_value = value
            self._initialized = True


class CircularAngleFilter:
    """
    环形角度滤波器 - 用于 Yaw 等 0°~360° 环形角度

    特点：
    - 将 0°~360° 转换为 -180°~+180° 进行处理
    - 正确处理 -180° 与 +180° 的等效性
    - 消除 0°/360° 跳变导致的滤波突变
    """

    def __init__(self, filter_coeff: float = 0.15):
        self.filter_coeff = filter_coeff
        self._last_value: float = float('nan')
        self._initialized: bool = False

    def filter(self, raw_value: float) -> float:
        """
        对环形角度进行低通滤波

        Args:
            raw_value: 原始角度 (0°~360°)

        Returns:
            滤波后的角度 (-180°~+180°)
        """
        # 转换为 -180°~+180°
        wrapped = self._normalize_to_signed(raw_value)

        if not self._initialized:
            self._last_value = wrapped
            self._initialized = True
            return wrapped

        # 处理 0°/360° 环形边界跳变
        # 当从正数跳到负数（接近 0°）或反之时，选择最短路径
        diff = wrapped - self._last_value
        if diff > 180.0:
            wrapped -= 360.0
        elif diff < -180.0:
            wrapped += 360.0

        # 标准指数平滑公式
        filtered = self.filter_coeff * wrapped + (1 - self.filter_coeff) * self._last_value
        self._last_value = filtered

        return filtered

    @staticmethod
    def _normalize_to_signed(angle: float) -> float:
        """将 0°~360° 角度转换为 -180°~+180°"""
        normalized = angle % 360.0
        if normalized > 180.0:
            normalized -= 360.0
        return normalized

    @staticmethod
    def _shortest_angle_diff(from_angle: float, to_angle: float) -> float:
        """
        计算两个角度间的最短路径差角

        Args:
            from_angle: 起始角度
            to_angle: 目标角度

        Returns:
            最短路径差角（范围 -180°~+180°）
        """
        diff = to_angle - from_angle
        # 归一化到 -180°~+180°
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0
        return diff

    def reset(self, value: float = None):
        """重置滤波器状态"""
        self._initialized = False
        if value is not None:
            self._last_value = self._normalize_to_signed(value)
            self._initialized = True


class TripleAxisFilter:
    """
    三轴角度综合滤波器

    一次性处理 Roll、Pitch、Yaw 三个轴的角度数据
    """

    def __init__(
        self,
        yaw_coeff: float = 0.15,
        pitch_coeff: float = 0.15,
        roll_coeff: float = 0.15
    ):
        self.yaw_filter = CircularAngleFilter(filter_coeff=yaw_coeff)
        self.pitch_filter = AngleFilter(filter_coeff=pitch_coeff)
        self.roll_filter = AngleFilter(filter_coeff=roll_coeff)

        self._roll_limit: Tuple[float, float] = (-90.0, 90.0)
        self._pitch_limit: Tuple[float, float] = (-90.0, 90.0)

    def filter(
        self,
        raw_roll: float,
        raw_pitch: float,
        raw_yaw: float
    ) -> Tuple[float, float, float]:
        """
        一次性过滤三个轴的角度

        Args:
            raw_roll: 横滚角原始值 (°)
            raw_pitch: 俯仰角原始值 (°)
            raw_yaw: 航向角原始值 (0°~360°)

        Returns:
            (filtered_roll, filtered_pitch, filtered_yaw)
            - roll: 线性角度 (-90°~+90°)
            - pitch: 线性角度 (-90°~+90°)
            - yaw: 环形角度 (-180°~+180°)
        """
        # Roll: 线性角度，直接限幅 + 滤波
        roll_clamped = self._clamp_linear(raw_roll, self._roll_limit)
        filtered_roll = self.roll_filter.filter(roll_clamped)

        # Pitch: 线性角度，直接限幅 + 滤波
        pitch_clamped = self._clamp_linear(raw_pitch, self._pitch_limit)
        filtered_pitch = self.pitch_filter.filter(pitch_clamped)

        # Yaw: 环形角度，滤波后保持 -180°~+180°
        filtered_yaw = self.yaw_filter.filter(raw_yaw)

        return filtered_roll, filtered_pitch, filtered_yaw

    def filter_yaw_only(self, raw_yaw: float) -> float:
        """仅过滤 Yaw 角度"""
        return self.yaw_filter.filter(raw_yaw)

    @staticmethod
    def _clamp_linear(value: float, limits: Tuple[float, float]) -> float:
        """限幅归一化"""
        return max(limits[0], min(limits[1], value))

    def reset(self):
        """重置所有滤波器状态"""
        self.yaw_filter.reset()
        self.pitch_filter.reset()
        self.roll_filter.reset()

    def set_coefficients(self, yaw_coeff: float = None,
                        pitch_coeff: float = None, roll_coeff: float = None):
        """动态调整滤波系数"""
        if yaw_coeff is not None:
            self.yaw_filter.filter_coeff = max(0.0, min(1.0, yaw_coeff))
        if pitch_coeff is not None:
            self.pitch_filter.filter_coeff = max(0.0, min(1.0, pitch_coeff))
        if roll_coeff is not None:
            self.roll_filter.filter_coeff = max(0.0, min(1.0, roll_coeff))

    def set_limits(self, roll_limit: Tuple[float, float] = None,
                   pitch_limit: Tuple[float, float] = None):
        """设置角度限幅范围"""
        if roll_limit is not None:
            self._roll_limit = roll_limit
        if pitch_limit is not None:
            self._pitch_limit = pitch_limit


# ========== 快捷函数 ==========

# 全局单例滤波器
_global_filter: TripleAxisFilter = None


def _get_global_filter(yaw_coeff: float = 0.15,
                       pitch_coeff: float = 0.15,
                       roll_coeff: float = 0.15) -> TripleAxisFilter:
    """获取或创建全局滤波器实例"""
    global _global_filter
    if _global_filter is None:
        _global_filter = TripleAxisFilter(yaw_coeff, pitch_coeff, roll_coeff)
    return _global_filter


def filter_angles(
    raw_roll: float,
    raw_pitch: float,
    raw_yaw: float,
    yaw_coeff: float = 0.15,
    pitch_coeff: float = 0.15,
    roll_coeff: float = 0.15
) -> Tuple[float, float, float]:
    """
    快捷函数：过滤三轴角度

    Args:
        raw_roll: 横滚角原始值 (°)
        raw_pitch: 俯仰角原始值 (°)
        raw_yaw: 航向角原始值 (0°~360°)
        yaw_coeff: Yaw 滤波系数
        pitch_coeff: Pitch 滤波系数
        roll_coeff: Roll 滤波系数

    Returns:
        (filtered_roll, filtered_pitch, filtered_yaw)

    Example:
        roll, pitch, yaw = filter_angles(10.5, -5.2, 359.8)
        print(f"Yaw: {yaw:.1f}°")
    """
    filter_obj = _get_global_filter(yaw_coeff, pitch_coeff, roll_coeff)
    return filter_obj.filter(raw_roll, raw_pitch, raw_yaw)


def filter_yaw(raw_yaw: float, filter_coeff: float = 0.15) -> float:
    """
    快捷函数：仅过滤 Yaw 角度

    Args:
        raw_yaw: 航向角原始值 (0°~360°)
        filter_coeff: 滤波系数

    Returns:
        滤波后的 Yaw 角度 (-180°~+180°)
    """
    filter_obj = _get_global_filter()
    filter_obj.set_coefficients(yaw_coeff=filter_coeff)
    return filter_obj.filter_yaw_only(raw_yaw)


def yaw_to_display(yaw_signed: float) -> float:
    """
    将 -180°~+180° 的 Yaw 角度转换回 0°~360° 显示格式

    Args:
        yaw_signed: 有符号角度 (-180°~+180°)

    Returns:
        显示角度 (0°~360°)
    """
    yaw = yaw_signed % 360.0
    if yaw < 0:
        yaw += 360.0
    return yaw


def reset_filters():
    """重置全局滤波器状态"""
    global _global_filter
    if _global_filter:
        _global_filter.reset()


class FilteredAngleData(NamedTuple):
    """滤波后的角度数据结构"""
    roll: float      # 横滚角 (线性, -90°~+90°)
    pitch: float     # 俯仰角 (线性, -90°~+90°)
    yaw: float       # 航向角 (显示用, 0°~360°)
    yaw_signed: float  # 航向角 (计算用, -180°~+180°)


def filter_angles_ex(
    raw_roll: float,
    raw_pitch: float,
    raw_yaw: float,
    yaw_coeff: float = 0.15,
    pitch_coeff: float = 0.15,
    roll_coeff: float = 0.15
) -> FilteredAngleData:
    """
    扩展函数：过滤三轴角度并返回完整数据结构

    Args:
        raw_roll: 横滚角原始值 (°)
        raw_pitch: 俯仰角原始值 (°)
        raw_yaw: 航向角原始值 (0°~360°)
        yaw_coeff: Yaw 滤波系数
        pitch_coeff: Pitch 滤波系数
        roll_coeff: Roll 滤波系数

    Returns:
        FilteredAngleData
            - roll: 横滚角 (线性)
            - pitch: 俯仰角 (线性)
            - yaw: 航向角 (0°~360° 显示用)
            - yaw_signed: 航向角 (-180°~+180° 计算用)
    """
    roll, pitch, yaw_signed = filter_angles(
        raw_roll, raw_pitch, raw_yaw,
        yaw_coeff, pitch_coeff, roll_coeff
    )
    return FilteredAngleData(
        roll=roll,
        pitch=pitch,
        yaw=yaw_to_display(yaw_signed),
        yaw_signed=yaw_signed
    )


# ========== 与 DDM350B 集成 ==========

def process_ddm350b_data(compass_data) -> FilteredAngleData:
    """
    处理 DDM350B 罗盘数据的便捷函数

    Args:
        compass_data: DDM350B.read() 返回的 CompassData 对象

    Returns:
        FilteredAngleData: 滤波后的角度数据

    Example:
        from ddm350b import DDM350B
        from angle_filter import process_ddm350b_data

        compass = DDM350B('/dev/ttyS0')
        compass.connect()

        data = compass.read()
        filtered = process_ddm350b_data(data)
        print(f"Yaw: {filtered.yaw:.1f}°  Pitch: {filtered.pitch:.1f}°  Roll: {filtered.roll:.1f}°")
    """
    return filter_angles_ex(
        raw_roll=compass_data.roll,
        raw_pitch=compass_data.pitch,
        raw_yaw=compass_data.heading
    )