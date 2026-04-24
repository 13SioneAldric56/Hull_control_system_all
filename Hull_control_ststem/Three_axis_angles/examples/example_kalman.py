"""
DDM350B/DDM360B 卡尔曼滤波示例

演示如何使用 compass_kalman 模块对三轴罗盘数据进行滤波

使用方法:
    python example_kalman.py

按 Ctrl+C 退出程序
"""

import sys
import time
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ddm350b import DDM350B
from compass_kalman import CompassKalmanFilter, FilteredCompassData


def format_bar(value: float, max_val: float = 10.0, width: int = 20) -> str:
    """将值转换为可视化柱状图"""
    normalized = max(-1, min(1, value / max_val))
    filled = int(abs(normalized) * width)
    bar = '=' * filled
    if normalized < 0:
        return '<' + bar + ' ' * (width - filled) + '|'
    else:
        return '|' + ' ' * (width - filled) + bar + '>'


def run_kalman_demo(port: str = '/dev/ttyS0', interval: float = 0.1):
    """
    运行卡尔曼滤波演示

    Args:
        port: 串口名
        interval: 读取间隔（秒）
    """
    print("=" * 60)
    print("DDM350B 卡尔曼滤波演示")
    print("=" * 60)
    print(f"串口: {port}")
    print(f"读取间隔: {interval}s")
    print("-" * 60)

    from ddm350b import OutputMode

    compass = DDM350B(port, timeout=2.0)
    if not compass.connect():
        print(f"错误: 无法连接到 {port}")
        print("请检查:")
        print("  1. 串口连接是否正常")
        print("  2. 串口是否被其他程序占用")
        print("  3. 波特率设置是否正确 (115200)")
        return

    print(f"已连接到 {port}")

    compass.set_mode(OutputMode.AUTO_50HZ)
    print("模式: 自动输出 50Hz")

    print("正在初始化滤波器...")
    kf = CompassKalmanFilter(
        process_noise=0.001,
        measurement_noise=0.1,
        wrap_angles=[False, False, True]
    )

    print("初始化完成，开始读取数据...")
    print("-" * 60)

    sample_count = 0
    consecutive_failures = 0
    max_consecutive_failures = 20
    start_time = time.time()

    try:
        while True:
            raw_data = compass.read()

            if raw_data is None:
                consecutive_failures += 1
                if consecutive_failures == 1:
                    print("[等待数据...]")
                elif consecutive_failures > max_consecutive_failures:
                    print("[警告] 连续读取失败次数过多，检查传感器连接")
                    consecutive_failures = 0
                time.sleep(0.1)
                continue

            consecutive_failures = 0
            filtered = kf.update(raw_data)

            sample_count += 1
            elapsed = time.time() - start_time

            roll_std = filtered.roll_std
            pitch_std = filtered.pitch_std
            heading_std = filtered.heading_std

            print(f"\n[样本 {sample_count}] 耗时: {elapsed:.1f}s")
            print(f"  横滚角 Roll:")
            print(f"    原始值: {raw_data.roll:>+8.2f}°")
            print(f"    滤波值: {filtered.roll:>+8.2f}°  ±{roll_std:.4f}°")
            print(f"    可视化: {format_bar(filtered.roll, max_val=45)}")

            print(f"  俯仰角 Pitch:")
            print(f"    原始值: {raw_data.pitch:>+8.2f}°")
            print(f"    滤波值: {filtered.pitch:>+8.2f}°  ±{pitch_std:.4f}°")
            print(f"    可视化: {format_bar(filtered.pitch, max_val=45)}")

            print(f"  航向角 Heading:")
            print(f"    原始值: {raw_data.heading:>8.2f}°")
            print(f"    滤波值: {filtered.heading:>8.2f}°  ±{heading_std:.4f}°")

            print("-" * 60)

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n\n程序已停止")
    finally:
        compass.disconnect()
        print(f"总共处理 {sample_count} 个样本")


def run_adaptive_demo(port: str = '/dev/ttyS0', interval: float = 0.1):
    """
    运行自适应卡尔曼滤波演示

    Args:
        port: 串口名
        interval: 读取间隔（秒）
    """
    from compass_kalman import AdaptiveCompassKalmanFilter

    print("=" * 60)
    print("自适应卡尔曼滤波演示")
    print("=" * 60)

    compass = DDM350B(port)
    if not compass.connect():
        print(f"错误: 无法连接到 {port}")
        return

    print("已连接")

    kf = AdaptiveCompassKalmanFilter(
        process_noise=0.001,
        measurement_noise=0.1,
        adaptation_rate=0.1,
        min_measurement_noise=0.05,
        max_measurement_noise=1.0
    )

    print("自适应滤波器已初始化")
    print("-" * 60)

    sample_count = 0
    last_measurement_noise = 0.0

    try:
        while True:
            raw_data = compass.read()
            if raw_data is None:
                time.sleep(0.05)
                continue

            filtered = kf.update(raw_data)

            sample_count += 1

            current_noise = kf._roll_kf.measurement_noise
            noise_changed = abs(current_noise - last_measurement_noise) > 0.001

            print(f"\n[样本 {sample_count}]")
            if noise_changed:
                print(f"  自适应噪声: {current_noise:.4f} {'↑' if current_noise > last_measurement_noise else '↓'}")
            else:
                print(f"  自适应噪声: {current_noise:.4f} (稳定)")

            print(f"  原始: R={raw_data.roll:>+7.2f} P={raw_data.pitch:>+7.2f} H={raw_data.heading:>7.2f}")
            print(f"  滤波: R={filtered.roll:>+7.2f} P={filtered.pitch:>+7.2f} H={filtered.heading:>7.2f}")

            print("-" * 60)
            last_measurement_noise = current_noise
            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n\n程序已停止")
    finally:
        compass.disconnect()


def run_compare_demo(port: str = '/dev/ttyS0', count: int = 50, interval: float = 0.1):
    """
    对比原始数据和滤波后数据

    Args:
        port: 串口名
        count: 采样数量
        interval: 读取间隔
    """
    import numpy as np

    print("=" * 60)
    print("滤波效果对比测试")
    print("=" * 60)

    compass = DDM350B(port)
    if not compass.connect():
        print(f"错误: 无法连接到 {port}")
        return

    print(f"正在采集 {count} 个样本进行对比分析...")

    kf = CompassKalmanFilter(process_noise=0.001, measurement_noise=0.1)

    raw_roll, raw_pitch, raw_heading = [], [], []
    filt_roll, filt_pitch, filt_heading = [], [], []

    try:
        for i in range(count):
            raw_data = compass.read()
            if raw_data is None:
                time.sleep(0.05)
                continue

            filtered = kf.update(raw_data)

            raw_roll.append(raw_data.roll)
            raw_pitch.append(raw_data.pitch)
            raw_heading.append(raw_data.heading)

            filt_roll.append(filtered.roll)
            filt_pitch.append(filtered.pitch)
            filt_heading.append(filtered.heading)

            print(f"\r采集进度: {i+1}/{count}", end='', flush=True)
            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n采集被中断")
    finally:
        compass.disconnect()

    print("\n\n" + "=" * 60)
    print("统计结果")
    print("=" * 60)

    raw_roll = np.array(raw_roll)
    raw_pitch = np.array(raw_pitch)
    raw_heading = np.array(raw_heading)
    filt_roll = np.array(filt_roll)
    filt_pitch = np.array(filt_pitch)
    filt_heading = np.array(filt_heading)

    def calc_std(arr):
        return np.std(arr)

    def calc_range(arr):
        return np.max(arr) - np.min(arr)

    print("\n横滚角 Roll:")
    print(f"  原始标准差: {calc_std(raw_roll):.4f}°  范围: {calc_range(raw_roll):.2f}°")
    print(f"  滤波标准差: {calc_std(filt_roll):.4f}°  范围: {calc_range(filt_roll):.2f}°")
    print(f"  噪声降低: {(1 - calc_std(filt_roll)/calc_std(raw_roll))*100:.1f}%")

    print("\n俯仰角 Pitch:")
    print(f"  原始标准差: {calc_std(raw_pitch):.4f}°  范围: {calc_range(raw_pitch):.2f}°")
    print(f"  滤波标准差: {calc_std(filt_pitch):.4f}°  范围: {calc_range(filt_pitch):.2f}°")
    print(f"  噪声降低: {(1 - calc_std(filt_pitch)/calc_std(raw_pitch))*100:.1f}%")

    print("\n航向角 Heading:")
    print(f"  原始标准差: {calc_std(raw_heading):.4f}°  范围: {calc_range(raw_heading):.2f}°")
    print(f"  滤波标准差: {calc_std(filt_heading):.4f}°  范围: {calc_range(filt_heading):.2f}°")
    print(f"  噪声降低: {(1 - calc_std(filt_heading)/calc_std(raw_heading))*100:.1f}%")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='DDM350B 卡尔曼滤波示例')
    parser.add_argument('-p', '--port', default='/dev/ttyS0', help='串口名 (默认: /dev/ttyS0)')
    parser.add_argument('-i', '--interval', type=float, default=0.1, help='读取间隔秒数 (默认: 0.1)')
    parser.add_argument('-m', '--mode', choices=['kalman', 'adaptive', 'compare'],
                        default='kalman', help='运行模式')
    parser.add_argument('-c', '--count', type=int, default=50, help='对比模式采样数')

    args = parser.parse_args()

    if args.mode == 'kalman':
        run_kalman_demo(args.port, args.interval)
    elif args.mode == 'adaptive':
        run_adaptive_demo(args.port, args.interval)
    elif args.mode == 'compare':
        run_compare_demo(args.port, args.count, args.interval)
