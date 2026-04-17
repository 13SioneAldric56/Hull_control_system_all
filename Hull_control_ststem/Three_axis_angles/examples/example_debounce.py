#!/usr/bin/env python3
"""
DDM350B/DDM360B 消抖滤波示例

对比卡尔曼滤波与简单消抖滤波（指数移动平均）的效果

使用方法:
    python example_debounce.py -p COM3

按 Ctrl+C 退出程序
"""

import sys
import time
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ddm350b import DDM350B, OutputMode
from collections import deque


class DebounceFilter:
    """
    简单的指数移动平均消抖滤波器

    使用方法：alpha越小，滤波越强，响应越慢
    alpha = 0.1 ~ 0.3 适合大多数应用
    alpha = 0.05 适合需要很强滤波的场景
    alpha > 0.5 几乎不滤波
    """

    def __init__(self, alpha: float = 0.2, wrap_heading: bool = True):
        """
        Args:
            alpha: 平滑系数 (0 < alpha <= 1), 越小越平滑
            wrap_heading: 是否对航向角进行角度回绕处理
        """
        self.alpha = alpha
        self.wrap_heading = wrap_heading
        self._initialized = False
        self._roll = 0.0
        self._pitch = 0.0
        self._heading = 0.0

    def update(self, roll: float, pitch: float, heading: float):
        """
        更新滤波值

        Args:
            roll, pitch, heading: 原始角度值

        Returns:
            (filtered_roll, filtered_pitch, filtered_heading)
        """
        if not self._initialized:
            self._roll = roll
            self._pitch = pitch
            self._heading = heading
            self._initialized = True
            return roll, pitch, heading

        # 普通角度直接使用指数移动平均
        self._roll = self.alpha * roll + (1 - self.alpha) * self._roll
        self._pitch = self.alpha * pitch + (1 - self.alpha) * self._pitch

        # 航向角需要处理0/360度的回绕问题
        if self.wrap_heading:
            self._heading = self._smooth_angle(self._heading, heading)
        else:
            self._heading = self.alpha * heading + (1 - self.alpha) * self._heading

        return self._roll, self._pitch, self._heading

    def _smooth_angle(self, current: float, target: float) -> float:
        """处理角度回绕的平滑"""
        diff = target - current
        # 取最短路径的差值
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return current + self.alpha * diff


class MovingAverageFilter:
    """
    移动平均滤波器 - 固定窗口大小
    """

    def __init__(self, window_size: int = 10, wrap_heading: bool = True):
        self.window_size = window_size
        self.wrap_heading = wrap_heading
        self._roll_buffer = deque(maxlen=window_size)
        self._pitch_buffer = deque(maxlen=window_size)
        self._heading_buffer = deque(maxlen=window_size)

    def update(self, roll: float, pitch: float, heading: float):
        self._roll_buffer.append(roll)
        self._pitch_buffer.append(pitch)
        self._heading_buffer.append(heading)

        if len(self._roll_buffer) < 2:
            return roll, pitch, heading

        return (
            sum(self._roll_buffer) / len(self._roll_buffer),
            sum(self._pitch_buffer) / len(self._pitch_buffer),
            self._mean_angle(self._heading_buffer)
        )

    def _mean_angle(self, angles):
        """计算角度平均值（处理回绕）"""
        if not angles:
            return 0
        sin_sum = sum(math.sin(math.radians(a)) for a in angles)
        cos_sum = sum(math.cos(math.radians(a)) for a in angles)
        return math.degrees(math.atan2(sin_sum, cos_sum))


import math


def format_bar(value: float, max_val: float = 10.0, width: int = 15) -> str:
    """将值转换为可视化柱状图"""
    normalized = max(-1, min(1, value / max_val))
    filled = int(abs(normalized) * width)
    bar = '=' * filled
    if normalized < 0:
        return '<' + bar + ' ' * (width - filled) + '|'
    else:
        return '|' + ' ' * (width - filled) + bar + '>'


def run_debounce_demo(port: str = 'COM3', alpha: float = 0.2):
    """
    运行消抖滤波演示

    Args:
        port: 串口名
        alpha: 平滑系数，越小越平滑
    """
    print("=" * 65)
    print("DDM350B 消抖滤波演示（指数移动平均 EMA）")
    print("=" * 65)
    print(f"串口: {port}")
    print(f"平滑系数 alpha: {alpha} (越小越平滑)")
    print("-" * 65)

    compass = DDM350B(port, timeout=2.0)
    if not compass.connect():
        print(f"错误: 无法连接到 {port}")
        return

    print(f"已连接到 {port}")
    compass.set_mode(OutputMode.AUTO_50HZ)
    print("模式: 自动输出 50Hz")
    print("-" * 65)

    debounce = DebounceFilter(alpha=alpha, wrap_heading=True)

    sample_count = 0
    consecutive_failures = 0
    start_time = time.time()

    try:
        while True:
            raw_data = compass.read()

            if raw_data is None:
                consecutive_failures += 1
                if consecutive_failures == 1:
                    print("[等待数据...]")
                time.sleep(0.01)
                continue

            consecutive_failures = 0
            filt_roll, filt_pitch, filt_heading = debounce.update(
                raw_data.roll, raw_data.pitch, raw_data.heading
            )

            sample_count += 1
            elapsed = time.time() - start_time

            print(f"\n[样本 {sample_count}] 耗时: {elapsed:.1f}s")

            print(f"  横滚角 Roll:")
            print(f"    原始: {raw_data.roll:>+8.2f}°  |  消抖: {filt_roll:>+8.2f}°  {format_bar(filt_roll, max_val=45)}")

            print(f"  俯仰角 Pitch:")
            print(f"    原始: {raw_data.pitch:>+8.2f}°  |  消抖: {filt_pitch:>+8.2f}°  {format_bar(filt_pitch, max_val=45)}")

            print(f"  航向角 Heading:")
            print(f"    原始: {raw_data.heading:>8.2f}°  |  消抖: {filt_heading:>8.2f}°")

            print("-" * 65)
            time.sleep(0.02)  # 约50Hz

    except KeyboardInterrupt:
        print("\n\n程序已停止")
    finally:
        compass.disconnect()
        print(f"总共处理 {sample_count} 个样本")


def run_compare_demo(port: str = 'COM3', count: int = 100, alpha: float = 0.2):
    """
    对比卡尔曼滤波与消抖滤波的效果

    Args:
        port: 串口名
        count: 采样数量
        alpha: 消抖系数
    """
    from compass_kalman import CompassKalmanFilter
    import numpy as np

    print("=" * 65)
    print("卡尔曼滤波 vs 消抖滤波 对比测试")
    print("=" * 65)
    print(f"串口: {port}")
    print(f"采样数量: {count}")
    print(f"消抖系数 alpha: {alpha}")
    print("-" * 65)

    compass = DDM350B(port, timeout=2.0)
    if not compass.connect():
        print(f"错误: 无法连接到 {port}")
        return

    print(f"已连接到 {port}")
    compass.set_mode(OutputMode.AUTO_50HZ)
    print("模式: 自动输出 50Hz")
    print(f"正在采集 {count} 个样本...\n")

    # 初始化滤波器
    debounce = DebounceFilter(alpha=alpha, wrap_heading=True)
    kalman = CompassKalmanFilter(
        process_noise=0.001,
        measurement_noise=0.1,
        wrap_angles=[False, False, True]
    )

    raw_roll, raw_pitch, raw_heading = [], [], []
    deb_roll, deb_pitch, deb_heading = [], [], []
    kal_roll, kal_pitch, kal_heading = [], [], []

    try:
        for i in range(count):
            raw_data = compass.read()
            if raw_data is None:
                time.sleep(0.01)
                continue

            # 三种滤波
            d_roll, d_pitch, d_heading = debounce.update(
                raw_data.roll, raw_data.pitch, raw_data.heading
            )
            k_data = kalman.update(raw_data)

            raw_roll.append(raw_data.roll)
            raw_pitch.append(raw_data.pitch)
            raw_heading.append(raw_data.heading)
            deb_roll.append(d_roll)
            deb_pitch.append(d_pitch)
            deb_heading.append(d_heading)
            kal_roll.append(k_data.roll)
            kal_pitch.append(k_data.pitch)
            kal_heading.append(k_data.heading)

            print(f"\r采集进度: {len(raw_roll)}/{count}", end='', flush=True)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n采集被中断")
    finally:
        compass.disconnect()

    print("\n\n" + "=" * 65)
    print("统计结果")
    print("=" * 65)

    def calc_std(arr):
        return np.std(arr)

    def calc_range(arr):
        return np.max(arr) - np.min(arr)

    results = []

    for name, raw, deb, kal in [
        ("横滚角 Roll", raw_roll, deb_roll, kal_roll),
        ("俯仰角 Pitch", raw_pitch, deb_pitch, kal_pitch),
        ("航向角 Heading", raw_heading, deb_heading, kal_heading)
    ]:
        print(f"\n{name}:")
        print(f"  原始数据    标准差: {calc_std(raw):.4f}°  范围: {calc_range(raw):.2f}°")
        print(f"  消抖滤波    标准差: {calc_std(deb):.4f}°  范围: {calc_range(deb):.2f}°  "
              f"降低: {(1-calc_std(deb)/calc_std(raw))*100:.1f}%")
        print(f"  卡尔曼滤波  标准差: {calc_std(kal):.4f}°  范围: {calc_range(kal):.2f}°  "
              f"降低: {(1-calc_std(kal)/calc_std(raw))*100:.1f}%")

        results.append({
            'name': name,
            'raw_std': calc_std(raw),
            'deb_std': calc_std(deb),
            'kal_std': calc_std(kal)
        })

    print("\n" + "=" * 65)
    print("总结")
    print("=" * 65)
    for r in results:
        print(f"{r['name']}: 消抖降噪 {r['deb_std']/r['raw_std']:.1%} | "
              f"卡尔曼降噪 {r['kal_std']/r['raw_std']:.1%}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='DDM350B 消抖滤波示例')
    parser.add_argument('-p', '--port', default='COM3', help='串口名 (默认: COM3)')
    parser.add_argument('-a', '--alpha', type=float, default=0.2,
                        help='平滑系数 0.05-0.5, 越小越平滑 (默认: 0.2)')
    parser.add_argument('-m', '--mode', choices=['debounce', 'compare'],
                        default='debounce', help='运行模式')
    parser.add_argument('-c', '--count', type=int, default=100, help='对比模式采样数')

    args = parser.parse_args()

    if args.mode == 'debounce':
        run_debounce_demo(args.port, args.alpha)
    elif args.mode == 'compare':
        run_compare_demo(args.port, args.count, args.alpha)
