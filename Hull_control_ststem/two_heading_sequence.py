#!/usr/bin/env python3
"""
分段航向锁定：先保持指定航向运行一段时长，再目标航向 +180° 运行同样时长。

依赖 heading_lock_control.HeadingLockController，不修改原模块。
启动失败时若在航向回环模式下会误对空的 _compass 调用 disconnect，
本脚本在导入后对 HeadingLockController.start 做一次进程内猴子补丁并正确释放罗盘资源。
"""

from __future__ import annotations

import argparse
import sys
import time

from heading_lock_control import HeadingLockController
from Three_axis_angles.ddm350b import OutputMode


def _release_compass_on_init_failure(ctrl: HeadingLockController) -> None:
    """航向回环模式下 _compass 可能为 None，原 start() 失败分支会误调 disconnect。"""
    if ctrl._compass is not None:
        try:
            ctrl._compass.disconnect()
        except Exception:
            pass
        ctrl._compass = None
    if ctrl._heading_wrap_reader is not None:
        try:
            ctrl._heading_wrap_reader.stop()
        except Exception:
            pass
        ctrl._heading_wrap_reader = None


def _patched_heading_start(self, calibrate: bool = True) -> bool:
    if not self._init_compass():
        return False
    if not self._init_motor_driver():
        _release_compass_on_init_failure(self)
        return False
    if calibrate:
        if not self._calibrate_target_heading():
            _release_compass_on_init_failure(self)
            return False
    self._pid.reset()
    self._is_running = True
    self._last_update_time = time.time()
    print("[启动] 航向角锁定控制已启动 (Ctrl+C 停止)")
    print(f"[PID] Kp={self._pid.kp}, Ki={self._pid.ki}, Kd={self._pid.kd}")
    return True


HeadingLockController.start = _patched_heading_start  # 仅在本脚本进程内生效，不改源文件

MODE_MAP = {
    "polling": OutputMode.POLLING,
    "auto_5hz": OutputMode.AUTO_5HZ,
    "auto_15hz": OutputMode.AUTO_15HZ,
    "auto_25hz": OutputMode.AUTO_25HZ,
    "auto_35hz": OutputMode.AUTO_35HZ,
    "auto_50hz": OutputMode.AUTO_50HZ,
    "auto_100hz": OutputMode.AUTO_100HZ,
}

INTERVAL_MAP = {
    "polling": 0.1,
    "auto_5hz": 0.2,
    "auto_15hz": 0.067,
    "auto_25hz": 0.04,
    "auto_35hz": 0.029,
    "auto_50hz": 0.02,
    "auto_100hz": 0.01,
}


def build_controller(args: argparse.Namespace) -> HeadingLockController:
    m = args.mode
    return HeadingLockController(
        compass_port=args.port,
        base_speed=args.speed,
        deviation_threshold=args.threshold,
        pid_kp=args.kp,
        pid_ki=args.ki,
        pid_kd=args.kd,
        compass_mode=MODE_MAP[m],
        update_interval=INTERVAL_MAP[m],
        use_heading_wrap=not args.no_wrap,
    )


def run_segment(controller: HeadingLockController, heading_deg: float, duration: float) -> bool:
    if not controller.start(calibrate=False):
        print(
            "[错误] 控制器初始化失败（常见原因：GPIO 无写权限，可尝试 sudo 运行或配置 udev 规则）",
            file=sys.stderr,
        )
        return False
    controller.set_target_heading(heading_deg)
    controller.run_loop(duration=duration)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(
        description="指定航向运行 N 秒后，反转 180° 再运行 N 秒（每段单独启动控制器）。"
    )
    parser.add_argument(
        "--heading",
        type=float,
        required=True,
        help="第一段目标航向角 (度, 0–360)",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=3.0,
        help="每段运行时长 (秒)，默认 20",
    )
    parser.add_argument("-p", "--port", type=str, default="/dev/ttyS0", help="罗盘串口")
    parser.add_argument("-s", "--speed", type=int, default=80, help="基础速度 (0–100)")
    parser.add_argument("-t", "--threshold", type=float, default=10.0, help="偏差死区 (度)")
    parser.add_argument("--kp", type=float, default=2.0, help="PID Kp")
    parser.add_argument("--ki", type=float, default=0.05, help="PID Ki")
    parser.add_argument("--kd", type=float, default=0.8, help="PID Kd")
    parser.add_argument(
        "--mode",
        type=str,
        default="auto_50hz",
        choices=list(MODE_MAP.keys()),
        help="罗盘输出模式",
    )
    parser.add_argument("--no-wrap", action="store_true", help="禁用航向角回环处理")

    args = parser.parse_args()

    h1 = args.heading % 360.0
    h2 = (h1 + 180.0) % 360.0
    dur = args.duration

    print(f"[阶段 1] 目标航向 {h1:.1f}°，持续 {dur:.1f} s")
    if not run_segment(build_controller(args), h1, dur):
        sys.exit(1)

    print(f"[阶段 2] 目标航向 {h2:.1f}° (相对阶段 1 反转 180°)，持续 {dur:.1f} s")
    if not run_segment(build_controller(args), h2, dur):
        sys.exit(1)


if __name__ == "__main__":
    main()
