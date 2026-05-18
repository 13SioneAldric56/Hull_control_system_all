#!/usr/bin/env python3
"""
电子罗盘校准程序（设备内置校准）

使用 compass 包统一协议。示例:
    python compass_calibration.py
    python compass_calibration.py /dev/ttyS0 -b 115200
"""

import argparse
import sys
import time

try:
    import serial.tools.list_ports
except ImportError:
    serial = None

from compass import calibrate as compass_calibrate
from compass.config import CompassConfig
from compass.device import DDM350B

DEFAULT_PORT = "/dev/ttyS0"
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 1.0
CALIBRATION_DURATION = 50


def print_progress(remaining: int, total: int):
    pct = 100 * (total - remaining) / float(total)
    bar_len = 50
    filled = int(bar_len * (total - remaining) / total)
    bar = "█" * filled + "-" * (bar_len - filled)
    print(f"\r校准 |{bar}| {pct:.1f}% 剩余 {remaining:2d}s", end="", flush=True)


def compass_calibration(
    port: str = DEFAULT_PORT,
    baudrate: int = DEFAULT_BAUDRATE,
    duration: int = CALIBRATION_DURATION,
    dry_run: bool = False,
) -> bool:
    print(f"\n{'#' * 60}")
    print("#  电子罗盘 (DDM350B) 校准")
    print(f"#  串口: {port}  波特率: {baudrate}  时长: {duration}s")
    print(f"{'#' * 60}\n")

    if dry_run:
        for i in range(duration, 0, -1):
            print_progress(i, duration)
            time.sleep(0.05)
        print("\n[模拟] 校准完成")
        return True

    def on_tick(remaining: int):
        print_progress(remaining, duration)

    print("请在校准过程中做 8 字形或 360° 旋转…\n")
    ok = compass_calibrate(port=port, baudrate=baudrate, duration=duration, on_tick=on_tick)
    print("\n" + ("✓ 校准完成" if ok else "✗ 校准失败"))
    return ok


def interactive_calibration():
    port = input(f"串口 [{DEFAULT_PORT}]: ").strip() or DEFAULT_PORT
    baud_s = input(f"波特率 [{DEFAULT_BAUDRATE}]: ").strip()
    baudrate = int(baud_s) if baud_s else DEFAULT_BAUDRATE

    with DDM350B(CompassConfig(port=port, baudrate=baudrate)) as dev:
        print("\n命令: 1=开始校准  2=保存校准  3=读角度  q=退出")
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "q":
                break
            if cmd == "1":
                dev.start_calibration()
                print("已开始校准")
            elif cmd == "2":
                dev.save_calibration()
                print("已保存校准")
            elif cmd == "3":
                s = dev.read_full()
                print(s if s else "读取失败")
            else:
                print("无效命令")


def main():
    parser = argparse.ArgumentParser(description="DDM350B 电子罗盘校准")
    parser.add_argument("port", nargs="?", default=DEFAULT_PORT)
    parser.add_argument("-b", "--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("-d", "--duration", type=int, default=CALIBRATION_DURATION)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("-i", "--interactive", action="store_true")
    parser.add_argument("-l", "--list", action="store_true")
    args = parser.parse_args()

    if args.list and serial:
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device} - {p.description}")
        return

    if args.interactive:
        interactive_calibration()
        sys.exit(0)

    sys.exit(0 if compass_calibration(args.port, args.baudrate, args.duration, args.dry_run) else 1)


if __name__ == "__main__":
    main()
