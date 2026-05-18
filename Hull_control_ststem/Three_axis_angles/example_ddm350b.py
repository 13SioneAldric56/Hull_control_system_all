#!/usr/bin/env python3
"""
DDM350B 三轴罗盘示例（compass 包）

简单读取一次三轴:
    python example_ddm350b.py

指定串口 / 连续读取:
    python example_ddm350b.py -p /dev/ttyS0 -b 115200
    python example_ddm350b.py --continuous -n 20
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# 保证可导入 Hull_control_ststem/compass
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from compass import DDM350B, OutputMode, calibrate, read_compass
from compass.config import Axis, CompassConfig


def read_once(port: str, baudrate: int, mode: OutputMode) -> bool:
    """读取并打印当前三轴角度（一次）。"""
    sample = read_compass(port=port, baudrate=baudrate, mode=mode)
    if sample is None:
        print("读取失败，请检查串口与接线")
        return False
    print(f"Roll:    {sample.roll:>+8.2f}°")
    print(f"Pitch:   {sample.pitch:>+8.2f}°")
    print(f"Heading: {sample.heading:>8.2f}°")
    return True


def read_continuous(
    port: str,
    baudrate: int,
    mode: OutputMode,
    count: int,
    interval: float,
) -> None:
    cfg = CompassConfig(port=port, baudrate=baudrate, mode=mode, axis=Axis.ALL)
    compass = DDM350B(cfg)
    if not compass.connect():
        print(f"无法连接 {port}")
        return
    if mode != OutputMode.POLLING:
        compass.set_mode(mode)

    print(f"连续读取 ({count} 次, 模式={mode.name})")
    print("-" * 50)
    try:
        for i in range(count):
            sample = compass.read_full()
            if sample:
                print(
                    f"[{i + 1:3d}] "
                    f"Roll: {sample.roll:>+7.2f}°  "
                    f"Pitch: {sample.pitch:>+7.2f}°  "
                    f"Heading: {sample.heading:>7.2f}°"
                )
            else:
                print(f"[{i + 1:3d}] 读取失败")
            if interval > 0:
                time.sleep(interval)
    finally:
        compass.disconnect()


def run_calibration(port: str, baudrate: int, duration: int) -> None:
    print(f"开始设备校准 ({duration}s)，请旋转罗盘…")
    ok = calibrate(port=port, baudrate=baudrate, duration=duration)
    print("校准完成" if ok else "校准失败")


def main():
    parser = argparse.ArgumentParser(description="DDM350B 三轴读取示例")
    parser.add_argument("-p", "--port", default="/dev/ttyS0", help="串口")
    parser.add_argument("-b", "--baudrate", type=int, default=115200, help="波特率")
    parser.add_argument(
        "-m",
        "--mode",
        default="polling",
        choices=[m.name.lower() for m in OutputMode],
        help="输出模式",
    )
    parser.add_argument(
        "--continuous",
        action="store_true",
        help="连续读取（默认只读一次）",
    )
    parser.add_argument("-n", "--count", type=int, default=10, help="连续读取次数")
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="连续读取间隔(秒)",
    )
    parser.add_argument("--calibrate", action="store_true", help="执行设备磁校准")
    parser.add_argument(
        "--cal-duration",
        type=int,
        default=50,
        help="校准时长(秒)",
    )
    args = parser.parse_args()

    mode = OutputMode[args.mode.upper()]

    if args.calibrate:
        run_calibration(args.port, args.baudrate, args.cal_duration)
        return

    if args.continuous:
        read_continuous(
            args.port, args.baudrate, mode, args.count, args.interval
        )
    else:
        read_once(args.port, args.baudrate, mode)


if __name__ == "__main__":
    main()
