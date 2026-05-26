"""
SSH / 终端键盘遥控 — 双路电调（pwmchip ESC）

控制逻辑与 keyboard_remote_stdin.py 相同；底层为 esc_dual_drive。

用法: sudo python3 keyboard_remote_esc_stdin.py [--speed N]
"""
from __future__ import annotations

import argparse
import os
import select
import signal
import sys
import time
from pathlib import Path
from typing import Optional, Set

_car_dir = Path(__file__).resolve().parent
if str(_car_dir) not in sys.path:
    sys.path.insert(0, str(_car_dir))

from esc_dual_drive import EscDifferentialDrive, create_esc_dual_driver

MOTION_KEYS = frozenset({"w", "s", "a", "d"})
KEY_IDLE_SEC = 0.01
POLL_SEC = 0.01
STOP_DEBOUNCE_SEC = 0.02


def apply_motion(drive: EscDifferentialDrive, active: Set[str]) -> None:
    w, s, a, d = "w" in active, "s" in active, "a" in active, "d" in active

    if w and s:
        drive.stop()
        return
    if w:
        if a and not d:
            drive.differential_turn("差速左小弯")
        elif d and not a:
            drive.differential_turn("差速右小弯")
        else:
            drive.forward()
        return
    if s:
        drive.backward()
        return
    if a and d:
        drive.stop()
        return
    if a:
        drive.spin_left()
        return
    if d:
        drive.spin_right()
        return
    drive.stop()


def _compute_sig(active: Set[str]) -> tuple:
    w, s, a, d = "w" in active, "s" in active, "a" in active, "d" in active
    if w and s:
        return ("clash",)
    if w:
        if a ^ d:
            return ("w_turn", "L" if a else "R")
        return ("fwd",)
    if s:
        return ("back",)
    if a ^ d:
        return ("spin", "L" if a else "R")
    if not active & MOTION_KEYS:
        return ("stop",)
    return ("stop",)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="终端 WASD 双电调遥控 (pwmchip ESC)")
    p.add_argument(
        "--speed",
        "-s",
        type=int,
        default=50,
        metavar="N",
        help="初始整体速度 0～100，默认 50",
    )
    p.add_argument(
        "--no-unlock",
        action="store_true",
        help="跳过启动时双路 3s 中位解锁（已解锁过时可加此参数）",
    )
    return p.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(argv)
    base_speed = max(0, min(100, args.speed))

    drive: Optional[EscDifferentialDrive] = None
    fd = sys.stdin.fileno()
    old_term: Optional[list] = None

    def restore_tty() -> None:
        nonlocal old_term
        if old_term is not None:
            try:
                import termios

                termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)
            except (OSError, termios.error):
                pass
            old_term = None

    def on_signal(_sig, _frame) -> None:
        if drive is not None:
            drive.stop()
        restore_tty()
        sys.exit(0)

    try:
        import termios
        import tty
    except ImportError:
        print("需要 Unix 终端（termios/tty），当前环境不可用。")
        sys.exit(1)

    print("初始化双路电调 PWM，请确保螺旋桨/桨叶已拆除 …")
    drive = create_esc_dual_driver(
        base_speed=base_speed, auto_unlock=not args.no_unlock
    )
    drive.base_speed = base_speed

    last_seen: dict[str, float] = {}
    prev_sigint = signal.signal(signal.SIGINT, on_signal)

    old_term = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    verbose_diag = os.environ.get("KEYBOARD_REMOTE_DEBUG")

    if not sys.stdin.isatty():
        print(
            "[警告] stdin 不是交互式 TTY，键盘可能无效。请在本地终端前台运行。",
            file=sys.stderr,
        )

    print(
        "ESC 键盘遥控 | W/S 前进/后退 | A/D 原地转 | W+A/W+D 前进左/右弯\n"
        "空格/x 中位停 | +/- 调速(5) | [ / ] 调速(10) | 1～9→10～90% | 0→100%\n"
        f"松手: 键空闲≥{KEY_IDLE_SEC * 1000:.0f}ms 且无运动≥{STOP_DEBOUNCE_SEC * 1000:.0f}ms 才中位停\n"
        f"base_speed={drive.base_speed}% （--speed / -s 指定）| q 退出"
    )
    if verbose_diag:
        print(
            f"[KEYBOARD_REMOTE_DEBUG] stdin isatty={sys.stdin.isatty()} fd={fd}"
        )

    last_cmd: Optional[tuple] = None
    stop_idle_since: Optional[float] = None

    try:
        while True:
            speed_dirty = False

            if select.select([fd], [], [], POLL_SEC)[0]:
                try:
                    chunk_b = os.read(fd, 4096)
                except OSError:
                    chunk_b = b""
                if not chunk_b:
                    break
                chunk = chunk_b.decode("utf-8", errors="ignore")
                ts = time.monotonic()
                for ch in chunk:
                    cl = ch.lower()
                    if cl in ("q",):
                        raise SystemExit(0)
                    if ch == " " or cl == "x":
                        last_seen.clear()
                        stop_idle_since = None
                        if last_cmd != ("stop",):
                            drive.stop()
                            last_cmd = ("stop",)
                        continue
                    if cl == "+" or cl == "=":
                        drive.base_speed = min(100, drive.base_speed + 5)
                        speed_dirty = True
                        print(f"[速度] base_speed={drive.base_speed}%")
                        continue
                    if cl == "-" or cl == "_":
                        drive.base_speed = max(0, drive.base_speed - 5)
                        speed_dirty = True
                        print(f"[速度] base_speed={drive.base_speed}%")
                        continue
                    if ch == "[":
                        drive.base_speed = max(0, drive.base_speed - 10)
                        speed_dirty = True
                        print(f"[速度] base_speed={drive.base_speed}%")
                        continue
                    if ch == "]":
                        drive.base_speed = min(100, drive.base_speed + 10)
                        speed_dirty = True
                        print(f"[速度] base_speed={drive.base_speed}%")
                        continue
                    if cl in "123456789":
                        drive.base_speed = min(100, (ord(cl) - ord("0")) * 10)
                        speed_dirty = True
                        print(f"[速度] base_speed={drive.base_speed}%")
                        continue
                    if cl == "0":
                        drive.base_speed = 100
                        speed_dirty = True
                        print(f"[速度] base_speed={drive.base_speed}%")
                        continue
                    if cl in MOTION_KEYS:
                        last_seen[cl] = ts

            now = time.monotonic()
            active = {
                k
                for k, t in last_seen.items()
                if k in MOTION_KEYS and (now - t) <= KEY_IDLE_SEC
            }
            sig = _compute_sig(active)

            if sig != ("stop",):
                stop_idle_since = None
                if speed_dirty or sig != last_cmd:
                    apply_motion(drive, active)
                    last_cmd = sig
                continue

            if speed_dirty:
                speed_dirty = False
            if stop_idle_since is None:
                stop_idle_since = now
            if now - stop_idle_since < STOP_DEBOUNCE_SEC:
                continue

            if last_cmd is None:
                last_cmd = ("stop",)
            elif last_cmd != ("stop",):
                apply_motion(drive, active)
                last_cmd = ("stop",)

    finally:
        signal.signal(signal.SIGINT, prev_sigint)
        if drive is not None:
            drive.stop()
            drive.shutdown()
        restore_tty()


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        pass
