"""
DDM350B/DDM360B 三维电子罗盘卡尔曼滤波模块

为三轴角度数据提供卡尔曼滤波，消除传感器抖动和噪声

使用示例:
    # 方式1: 独立使用
    from compass_kalman import CompassKalmanFilter
    from ddm350b import DDM350B, CompassData

    kf = CompassKalmanFilter(process_noise=0.001, measurement_noise=0.1)
    compass = DDM350B('COM3')
    compass.connect()

    for _ in range(100):
        raw_data = compass.read()
        if raw_data:
            filtered = kf.update(raw_data)
            print(f"原始: R={raw_data.roll:.2f} P={raw_data.pitch:.2f} H={raw_data.heading:.2f}")
            print(f"滤波: R={filtered.roll:.2f} P={filtered.pitch:.2f} H={filtered.heading:.2f}")

    # 方式2: 带航向角角度 wrapping 处理
    kf = CompassKalmanFilter(process_noise=0.001, measurement_noise=0.1, wrap_angles=[False, False, True])

    # 方式3: 快捷函数
    from compass_kalman import filtered_read
    data = filtered_read('COM3', process_noise=0.001, measurement_noise=0.1)
"""

import numpy as np
from typing import Optional, List, NamedTuple
from dataclasses import dataclass


class FilteredCompassData(NamedTuple):
    """滤波后的罗盘数据结构"""
    roll: float      # 横滚角 (°)
    pitch: float     # 俯仰角 (°)
    heading: float   # 航向角 (°)
    roll_std: float  # 横滚角标准差估计
    pitch_std: float # 俯仰角标准差估计
    heading_std: float # 航向角标准差估计


@dataclass
class KalmanFilter1D:
    """
    一维卡尔曼滤波器

    用于单一标量测量值（如单轴角度）的滤波

    Attributes:
        process_noise: 过程噪声协方差 Q（越小表示模型越确定）
        measurement_noise: 测量噪声协方差 R（越小表示越信任测量值）
        initial_value: 初始估计值
        initial_covariance: 初始估计协方差
    """
    process_noise: float = 0.0001
    measurement_noise: float = 0.1
    initial_value: float = 0.0
    initial_covariance: float = 1.0

    def __post_init__(self):
        self._x = self.initial_value  # 状态估计
        self._p = self.initial_covariance  # 估计协方差
        self._initialized = False

    def reset(self, value: float = 0.0, covariance: float = 1.0):
        """重置滤波器状态"""
        self._x = value
        self._p = covariance
        self._initialized = False

    def update(self, measurement: float) -> float:
        """
        更新滤波器

        Args:
            measurement: 当前测量值

        Returns:
            滤波后的估计值
        """
        if not self._initialized:
            self._x = measurement
            self._initialized = True
            return self._x

        # 预测步骤
        x_pred = self._x
        p_pred = self._p + self.process_noise

        # 更新步骤
        k = p_pred / (p_pred + self.measurement_noise)
        self._x = x_pred + k * (measurement - x_pred)
        self._p = (1 - k) * p_pred

        return self._x

    def get_variance(self) -> float:
        """获取当前估计的方差"""
        return self._p

    def get_std(self) -> float:
        """获取当前估计的标准差"""
        return np.sqrt(self._p)


class CompassKalmanFilter:
    """
    三轴罗盘卡尔曼滤波器

    同时对 roll（横滚角）、pitch（俯仰角）、heading（航向角）进行卡尔曼滤波

    航向角自动处理 0-360° 的角度环绕问题

    Attributes:
        process_noise: 过程噪声协方差 Q（越小滤波越强，越慢跟踪）
        measurement_noise: 测量噪声协方差 R（越小越信任测量值）
        wrap_angles: 是否对各轴进行角度环绕处理 [roll, pitch, heading]
                     航向角默认开启环绕处理
    """

    def __init__(
        self,
        process_noise: float = 0.001,
        measurement_noise: float = 0.1,
        wrap_angles: Optional[List[bool]] = None
    ):
        if wrap_angles is None:
            wrap_angles = [False, False, True]

        self._roll_kf = KalmanFilter1D(
            process_noise=process_noise,
            measurement_noise=measurement_noise,
            initial_value=0.0
        )
        self._pitch_kf = KalmanFilter1D(
            process_noise=process_noise,
            measurement_noise=measurement_noise,
            initial_value=0.0
        )
        self._heading_kf = KalmanFilter1D(
            process_noise=process_noise,
            measurement_noise=measurement_noise,
            initial_value=0.0
        )

        self._wrap_angles = wrap_angles
        self._last_heading: Optional[float] = None

    def _wrap_angle_diff(self, new_angle: float, old_angle: float) -> float:
        """
        计算角度差，处理 0-360° 环绕

        确保选择最短路径的方向
        """
        diff = new_angle - old_angle
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    def update(self, data) -> FilteredCompassData:
        """
        更新滤波器并返回滤波后的数据

        Args:
            data: CompassData 或类似的三轴数据对象
                  支持 attributes: roll, pitch, heading

        Returns:
            FilteredCompassData: 滤波后的数据及标准差估计
        """
        roll = self._roll_kf.update(data.roll)
        pitch = self._pitch_kf.update(data.pitch)

        heading_raw = data.heading

        if self._wrap_angles[2]:
            if self._last_heading is not None:
                heading_diff = self._wrap_angle_diff(heading_raw, self._last_heading)
                target_heading = self._last_heading + heading_diff
                while target_heading < 0:
                    target_heading += 360
                while target_heading >= 360:
                    target_heading -= 360
                heading = self._heading_kf.update(target_heading)
            else:
                heading = self._heading_kf.update(heading_raw)
            self._last_heading = heading
        else:
            heading = self._heading_kf.update(heading_raw)

        return FilteredCompassData(
            roll=roll,
            pitch=pitch,
            heading=heading,
            roll_std=self._roll_kf.get_std(),
            pitch_std=self._pitch_kf.get_std(),
            heading_std=self._heading_kf.get_std()
        )

    def update_raw(self, roll: float, pitch: float, heading: float) -> FilteredCompassData:
        """
        使用原始数值更新滤波器

        Args:
            roll: 横滚角 (°)
            pitch: 俯仰角 (°)
            heading: 航向角 (°)

        Returns:
            FilteredCompassData: 滤波后的数据及标准差估计
        """
        class RawData:
            def __init__(self, r, p, h):
                self.roll = r
                self.pitch = p
                self.heading = h

        return self.update(RawData(roll, pitch, heading))

    def reset(self):
        """重置所有滤波器状态"""
        self._roll_kf.reset()
        self._pitch_kf.reset()
        self._heading_kf.reset()
        self._last_heading = None

    def set_noise(
        self,
        process_noise: Optional[float] = None,
        measurement_noise: Optional[float] = None
    ):
        """
        动态调整噪声参数

        Args:
            process_noise: 过程噪声协方差
            measurement_noise: 测量噪声协方差
        """
        if process_noise is not None:
            self._roll_kf.process_noise = process_noise
            self._pitch_kf.process_noise = process_noise
            self._heading_kf.process_noise = process_noise

        if measurement_noise is not None:
            self._roll_kf.measurement_noise = measurement_noise
            self._pitch_kf.measurement_noise = measurement_noise
            self._heading_kf.measurement_noise = measurement_noise


class AdaptiveCompassKalmanFilter(CompassKalmanFilter):
    """
    自适应三轴罗盘卡尔曼滤波器

    根据测量噪声自动调整滤波强度
    当测量值变化剧烈时降低滤波强度以快速跟踪
    当测量值稳定时增强滤波强度以减少噪声
    """

    def __init__(
        self,
        process_noise: float = 0.001,
        measurement_noise: float = 0.1,
        wrap_angles: Optional[List[bool]] = None,
        adaptation_rate: float = 0.1,
        min_measurement_noise: float = 0.05,
        max_measurement_noise: float = 1.0
    ):
        super().__init__(process_noise, measurement_noise, wrap_angles)
        self._adaptation_rate = adaptation_rate
        self._min_measurement_noise = min_measurement_noise
        self._max_measurement_noise = max_measurement_noise
        self._last_roll = None
        self._last_pitch = None
        self._last_heading_raw = None

    def update(self, data) -> FilteredCompassData:
        """带自适应噪声调整的更新"""
        roll_raw = data.roll
        pitch_raw = data.pitch
        heading_raw = data.heading

        roll_diff = abs(roll_raw - self._last_roll) if self._last_roll is not None else 0.0
        pitch_diff = abs(pitch_raw - self._last_pitch) if self._last_pitch is not None else 0.0
        heading_diff = abs(self._wrap_angle_diff(heading_raw, self._last_heading_raw)) if self._last_heading_raw is not None else 0.0

        total_diff = roll_diff + pitch_diff + heading_diff

        new_measurement_noise = max(
            self._min_measurement_noise,
            min(self._max_measurement_noise, total_diff * self._adaptation_rate * 10)
        )

        self.set_noise(measurement_noise=new_measurement_noise)

        self._last_roll = roll_raw
        self._last_pitch = pitch_raw
        self._last_heading_raw = heading_raw

        return super().update(data)


# ============ 快捷函数 ============

_global_filter: Optional[CompassKalmanFilter] = None
_global_compass = None


def filtered_read(
    port: str = 'COM3',
    process_noise: float = 0.001,
    measurement_noise: float = 0.1,
    use_adaptive: bool = False
) -> Optional[FilteredCompassData]:
    """
    快捷函数：读取并滤波一次罗盘数据

    Args:
        port: 串口名
        process_noise: 过程噪声
        measurement_noise: 测量噪声
        use_adaptive: 是否使用自适应滤波器

    Returns:
        FilteredCompassData 或 None
    """
    global _global_filter, _global_compass

    try:
        from ddm350b import DDM350B
    except ImportError:
        print("错误: 无法导入 ddm350b 模块")
        return None

    if _global_compass is None:
        _global_compass = DDM350B(port)
        if not _global_compass.connect():
            _global_compass = None
            return None

    if _global_compass.port != port:
        _global_compass.disconnect()
        _global_compass = DDM350B(port)
        if not _global_compass.connect():
            _global_compass = None
            return None

    raw_data = _global_compass.read()
    if raw_data is None:
        return None

    filter_class = AdaptiveCompassKalmanFilter if use_adaptive else CompassKalmanFilter

    if _global_filter is None:
        _global_filter = filter_class(
            process_noise=process_noise,
            measurement_noise=measurement_noise
        )

    return _global_filter.update(raw_data)


def reset_filter():
    """重置全局滤波器"""
    global _global_filter
    if _global_filter:
        _global_filter.reset()


def close_filter():
    """关闭全局滤波器并断开连接"""
    global _global_filter, _global_compass
    if _global_filter:
        _global_filter.reset()
    if _global_compass:
        _global_compass.disconnect()
        _global_compass = None


class ContinuousFilteredReader:
    """
    连续读取并滤波的生成器类

    使用示例:
        reader = ContinuousFilteredReader('COM3', process_noise=0.001)
        for filtered_data in reader:
            print(filtered_data.heading)
        reader.close()
    """

    def __init__(
        self,
        port: str = 'COM3',
        process_noise: float = 0.001,
        measurement_noise: float = 0.1,
        use_adaptive: bool = False,
        interval: float = 0.1
    ):
        try:
            from ddm350b import DDM350B
        except ImportError:
            raise ImportError("需要安装 ddm350b 模块")

        self._compass = DDM350B(port)
        if not self._compass.connect():
            raise ConnectionError(f"无法连接到 {port}")

        filter_class = AdaptiveCompassKalmanFilter if use_adaptive else CompassKalmanFilter
        self._filter = filter_class(
            process_noise=process_noise,
            measurement_noise=measurement_noise
        )
        self._interval = interval
        self._running = False

    def __iter__(self):
        return self

    def __next__(self) -> FilteredCompassData:
        import time

        if not self._running:
            self._running = True

        raw_data = self._compass.read()
        if raw_data is None:
            raise StopIteration

        filtered = self._filter.update(raw_data)
        time.sleep(self._interval)
        return filtered

    def close(self):
        """关闭 reader"""
        if self._compass:
            self._compass.disconnect()
            self._compass = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def continuous_filtered_read(
    port: str = 'COM3',
    count: Optional[int] = None,
    process_noise: float = 0.001,
    measurement_noise: float = 0.1,
    interval: float = 0.1
) -> List[FilteredCompassData]:
    """
    连续读取并滤波多组数据

    Args:
        port: 串口名
        count: 读取次数，None 表示无限
        process_noise: 过程噪声
        measurement_noise: 测量噪声
        interval: 读取间隔（秒）

    Returns:
        List[FilteredCompassData]

    Example:
        results = continuous_filtered_read('COM3', count=100)
    """
    import time
    try:
        from ddm350b import DDM350B
    except ImportError:
        print("错误: 无法导入 ddm350b 模块")
        return []

    compass = DDM350B(port)
    if not compass.connect():
        print(f"连接 {port} 失败")
        return []

    kf = CompassKalmanFilter(
        process_noise=process_noise,
        measurement_noise=measurement_noise
    )

    results = []
    idx = 0

    try:
        while count is None or idx < count:
            raw_data = compass.read()
            if raw_data:
                filtered = kf.update(raw_data)
                results.append(filtered)
                idx += 1
            time.sleep(interval)
    except KeyboardInterrupt:
        pass
    finally:
        compass.disconnect()

    return results
