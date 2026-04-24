"""
航向角回环处理模块

将罗盘输出的 0-360° 角度转换为连续角度（可超过 360°），用于：
1. PID 控制器的平滑输入
2. 航向角积分计算
3. 航向锁定系统的精确控制

功能特性:
1. 读取 DDM350B 罗盘原始数据
2. 卡尔曼滤波消除噪声
3. 回环处理：359° → 1° 变成 359° → 361°（连续无跳变）
4. 提供多种角度获取接口

使用示例:
    # 独立运行
    from heading_wrap import HeadingWrapReader

    reader = HeadingWrapReader('/dev/ttyS0')
    reader.start()
    for _ in range(100):
        heading = reader.get_heading()      # 连续角度 (可超过360°)
        wrapped = reader.get_wrapped_heading()  # 0-360° 角度
        print(f"连续: {heading:.1f}° | 环绕: {wrapped:.1f}°")
        time.sleep(0.1)

    # 获取原始卡尔曼滤波数据
    data = reader.get_filtered_data()
    print(f"横滚: {data.roll:.1f}° 俯仰: {data.pitch:.1f}°")
"""

import sys
import time
import math
from typing import Optional, NamedTuple
from dataclasses import dataclass

sys.path.insert(0, __file__.rsplit('/', 1)[0] if '/' in __file__ else '.')

from Three_axis_angles.compass_kalman import CompassKalmanFilter, FilteredCompassData
from Three_axis_angles.ddm350b import DDM350B, OutputMode


class WrappedHeadingData(NamedTuple):
    """回环处理后的航向数据结构"""
    continuous_heading: float  # 连续角度 (可超过360°)
    wrapped_heading: float     # 环绕角度 (0-360°)
    raw_heading: float        # 原始罗盘角度 (0-360°)
    roll: float               # 横滚角
    pitch: float              # 俯仰角
    heading_std: float        # 航向角标准差估计


class HeadingWrap:
    """
    航向角回环处理器

    将 0-360° 角度转换为连续角度，解决 0°/360° 边界的跳变问题

    算法原理:
    当检测到航向角从高值（如 350°）突变到低值（如 10°）时，
    不直接使用 10°，而是计算为 370° (350 + 20)，
    使角度保持连续可积。
    """

    def __init__(self, initial_heading: float = 0.0):
        """
        初始化回环处理器

        Args:
            initial_heading: 初始航向角 (0-360°)
        """
        self._continuous_heading: float = initial_heading
        self._last_wrapped_heading: float = initial_heading
        self._initialized: bool = False

    def reset(self, heading: float = 0.0):
        """重置回环状态"""
        self._continuous_heading = heading
        self._last_wrapped_heading = heading
        self._initialized = False

    def update(self, wrapped_heading: float) -> float:
        """
        更新航向角，返回连续角度

        Args:
            wrapped_heading: 罗盘输出的原始角度 (0-360°)

        Returns:
            连续航向角（可超过360°）
        """
        if not self._initialized:
            self._continuous_heading = wrapped_heading
            self._last_wrapped_heading = wrapped_heading
            self._initialized = True
            return self._continuous_heading

        diff = wrapped_heading - self._last_wrapped_heading

        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360

        self._continuous_heading += diff
        self._last_wrapped_heading = wrapped_heading

        return self._continuous_heading

    def get_continuous(self) -> float:
        """获取连续航向角"""
        return self._continuous_heading

    def get_wrapped(self) -> float:
        """获取环绕航向角 (0-360°)"""
        wrapped = self._continuous_heading % 360
        if wrapped < 0:
            wrapped += 360
        return wrapped

    def get_full_state(self) -> dict:
        """获取完整状态"""
        return {
            'continuous': self._continuous_heading,
            'wrapped': self.get_wrapped(),
            'last_wrapped': self._last_wrapped_heading
        }


class HeadingWrapReader:
    """
    航向角回环读取器

    整合罗盘读取、卡尔曼滤波和回环处理，提供统一的接口
    """

    def __init__(
        self,
        port: str = '/dev/ttyS0',
        process_noise: float = 0.001,
        measurement_noise: float = 0.1,
        compass_mode: OutputMode = OutputMode.AUTO_50HZ,
        use_adaptive: bool = False
    ):
        """
        初始化航向角读取器

        Args:
            port: 罗盘串口设备路径
            process_noise: 卡尔曼滤波过程噪声
            measurement_noise: 卡尔曼滤波测量噪声
            compass_mode: 罗盘输出模式
            use_adaptive: 是否使用自适应滤波器
        """
        self.port = port
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.compass_mode = compass_mode
        self.use_adaptive = use_adaptive

        self._compass: Optional[DDM350B] = None
        self._kalman: Optional[CompassKalmanFilter] = None
        self._heading_wrap: Optional[HeadingWrap] = None

        self._last_filtered: Optional[FilteredCompassData] = None
        self._is_running = False
        self._update_count = 0
        self._last_update_time: float = 0

    def start(self) -> bool:
        """
        启动读取器

        Returns:
            启动是否成功
        """
        try:
            self._compass = DDM350B(self.port)
            if not self._compass.connect():
                print(f"[错误] 无法连接到罗盘 {self.port}")
                return False

            if not self._compass.set_mode(self.compass_mode):
                print(f"[警告] 无法设置罗盘模式")

            if self.use_adaptive:
                from Three_axis_angles.compass_kalman import AdaptiveCompassKalmanFilter
                self._kalman = AdaptiveCompassKalmanFilter(
                    process_noise=self.process_noise,
                    measurement_noise=self.measurement_noise,
                    wrap_angles=[False, False, False]  # 关闭KF内部回环，使用我们的
                )
            else:
                self._kalman = CompassKalmanFilter(
                    process_noise=self.process_noise,
                    measurement_noise=self.measurement_noise,
                    wrap_angles=[False, False, False]  # 关闭KF内部回环，使用我们的
                )

            self._heading_wrap = HeadingWrap()

            self._is_running = True
            print(f"[启动] 航向角回环读取器已启动")
            print(f"  串口: {self.port}")
            print(f"  模式: {self.compass_mode.name if hasattr(self.compass_mode, 'name') else self.compass_mode}")
            print(f"  过程噪声: {self.process_noise}")
            print(f"  测量噪声: {self.measurement_noise}")
            return True

        except Exception as e:
            print(f"[错误] 启动失败: {e}")
            return False

    def update(self) -> Optional[WrappedHeadingData]:
        """
        更新一次数据

        Returns:
            回环处理后的数据，或 None（无数据）
        """
        if not self._is_running or self._compass is None:
            return None

        raw_data = self._compass.read()
        if raw_data is None:
            return None

        self._last_filtered = self._kalman.update(raw_data)

        continuous = self._heading_wrap.update(self._last_filtered.heading)
        wrapped = self._heading_wrap.get_wrapped()

        self._update_count += 1
        self._last_update_time = time.time()

        return WrappedHeadingData(
            continuous_heading=continuous,
            wrapped_heading=wrapped,
            raw_heading=raw_data.heading,
            roll=raw_data.roll,
            pitch=raw_data.pitch,
            heading_std=self._last_filtered.heading_std
        )

    def get_heading(self) -> Optional[float]:
        """
        获取连续航向角（可超过360°）

        Returns:
            连续航向角，或 None（无数据）
        """
        if self._heading_wrap is None:
            return None
        return self._heading_wrap.get_continuous()

    def get_wrapped_heading(self) -> Optional[float]:
        """
        获取环绕航向角 (0-360°)

        Returns:
            0-360° 航向角，或 None（无数据）
        """
        if self._heading_wrap is None:
            return None
        return self._heading_wrap.get_wrapped()

    def get_filtered_data(self) -> Optional[FilteredCompassData]:
        """
        获取卡尔曼滤波后的原始数据

        Returns:
            滤波后的数据（包含 roll, pitch, heading in 0-360°）
        """
        return self._last_filtered

    def get_full_data(self) -> Optional[WrappedHeadingData]:
        """
        获取完整回环处理数据

        Returns:
            包含所有角度信息的结构体
        """
        return self.update()

    def read_once(self) -> Optional[WrappedHeadingData]:
        """
        读取一次数据（更新并返回）

        Returns:
            回环处理后的数据
        """
        return self.update()

    def reset(self, heading: float = 0.0):
        """
        重置回环状态

        Args:
            heading: 新的初始航向角 (0-360°)
        """
        if self._heading_wrap:
            self._heading_wrap.reset(heading)
        if self._kalman:
            self._kalman.reset()
        self._update_count = 0
        print(f"[重置] 航向角已重置到 {heading:.1f}°")

    def stop(self):
        """停止读取器"""
        self._is_running = False
        if self._compass:
            self._compass.disconnect()
            self._compass = None
        print("[停止] 航向角读取器已关闭")

    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            'update_count': self._update_count,
            'last_update_time': self._last_update_time,
            'continuous_heading': self.get_heading(),
            'wrapped_heading': self.get_wrapped_heading(),
            'heading_wrap_state': self._heading_wrap.get_full_state() if self._heading_wrap else None
        }

    def __enter__(self):
        """支持 with 语句"""
        self.start()
        return self

    def __exit__(self, *args):
        """退出 with 语句时停止"""
        self.stop()


def heading_wrap_demo(duration: float = 30):
    """
    航向角回环演示

    展示回环处理如何消除 0°/360° 边界的跳变
    """
    print("\n" + "=" * 60)
    print("  航向角回环处理演示")
    print("=" * 60)
    print("\n演示内容：")
    print("  - 读取罗盘数据并滤波")
    print("  - 回环处理：将 0-360° 转换为连续角度")
    print("  - 展示跨越 0°/360° 边界时的连续性")
    print("\n按 Ctrl+C 停止\n")

    reader = HeadingWrapReader(
        port='/dev/ttyS0',
        compass_mode=OutputMode.AUTO_50HZ
    )

    if not reader.start():
        print("启动失败，请检查硬件连接")
        return

    start_time = time.time()
    raw_headings = []
    continuous_headings = []

    try:
        while True:
            data = reader.update()
            if data is None:
                time.sleep(0.05)
                continue

            elapsed = time.time() - start_time

            raw_headings.append(data.raw_heading)
            continuous_headings.append(data.continuous_heading)

            print("\033[2J\033[H")
            print("╔══════════════════════════════════════════════════════════════╗")
            print("║                    航向角回环处理演示                            ║")
            print("╠══════════════════════════════════════════════════════════════╣")
            print(f"║  运行时间: {elapsed:>6.1f}s                                           ║")
            print("╠══════════════════════════════════════════════════════════════╣")
            print("║  角度数据                                                       ║")
            print(f"║    ├─ 原始角度:    {data.raw_heading:>6.1f}° (0-360°)                        ║")
            print(f"║    ├─ 连续角度:    {data.continuous_heading:>6.1f}° (可超过360°)                ║")
            print(f"║    ├─ 环绕角度:    {data.wrapped_heading:>6.1f}° (0-360°)                        ║")
            print(f"║    └─ 横滚/俯仰:  {data.roll:>5.1f}° / {data.pitch:>5.1f}°                        ║")
            print("╠══════════════════════════════════════════════════════════════╣")
            print("║  变化检测                                                       ║")

            if len(raw_headings) >= 2:
                raw_diff = raw_headings[-1] - raw_headings[-2]
                con_diff = continuous_headings[-1] - continuous_headings[-2]

                if abs(raw_diff) > 180:
                    print(f"║    ⚠ 检测到边界跳变!                                              ║")
                    print(f"║      原始角度变化: {raw_diff:>+6.1f}°                                 ║")
                    print(f"║      连续角度变化: {con_diff:>+6.1f}° (已平滑)                         ║")

            print(f"║    └─ 累计更新: {reader._update_count} 次                                     ║")
            print("╚══════════════════════════════════════════════════════════════╝")

            if duration and elapsed >= duration:
                break

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n[停止] 用户中断")

    finally:
        reader.stop()

        if len(raw_headings) > 10:
            print("\n[统计] 航向角变化范围:")
            print(f"  原始角度: {min(raw_headings):.1f}° ~ {max(raw_headings):.1f}°")
            print(f"  连续角度: {min(continuous_headings):.1f}° ~ {max(continuous_headings):.1f}°")
            print(f"  总旋转量: {max(continuous_headings) - min(continuous_headings):.1f}°")


def angle_difference(angle1: float, angle2: float) -> float:
    """
    计算两个角度之间的最短差值（-180° ~ +180°）

    Args:
        angle1: 第一个角度 (°)
        angle2: 第二个角度 (°)

    Returns:
        角度差，范围 -180° ~ +180°
    """
    diff = angle2 - angle1
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff


def normalize_angle(angle: float) -> float:
    """
    将角度标准化到 0-360° 范围

    Args:
        angle: 任意角度

    Returns:
        0-360° 范围内的角度
    """
    wrapped = angle % 360
    if wrapped < 0:
        wrapped += 360
    return wrapped


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='航向角回环处理模块')
    parser.add_argument('-d', '--duration', type=float, default=30,
                        help='运行时长(秒)，默认30秒')
    parser.add_argument('-p', '--port', type=str, default='/dev/ttyS0',
                        help='罗盘串口路径，默认 /dev/ttyS0')
    parser.add_argument('--mode', type=str, default='auto_50hz',
                        choices=['polling', 'auto_5hz', 'auto_15hz', 'auto_25hz',
                                'auto_35hz', 'auto_50hz', 'auto_100hz'],
                        help='罗盘输出模式，默认auto_50hz')
    parser.add_argument('--adaptive', action='store_true',
                        help='使用自适应滤波器')

    args = parser.parse_args()

    mode_map = {
        'polling': OutputMode.POLLING,
        'auto_5hz': OutputMode.AUTO_5HZ,
        'auto_15hz': OutputMode.AUTO_15HZ,
        'auto_25hz': OutputMode.AUTO_25HZ,
        'auto_35hz': OutputMode.AUTO_35HZ,
        'auto_50hz': OutputMode.AUTO_50HZ,
        'auto_100hz': OutputMode.AUTO_100HZ,
    }

    reader = HeadingWrapReader(
        port=args.port,
        compass_mode=mode_map[args.mode],
        use_adaptive=args.adaptive
    )

    heading_wrap_demo(duration=args.duration)
