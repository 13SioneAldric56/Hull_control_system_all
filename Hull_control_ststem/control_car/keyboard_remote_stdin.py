"""

SSH / 终端键盘遥控差速车（stdin + termios）



无图形环境、无按键抬起事件：用「每键最后出现时间」+ 空闲超时近似松手。

停止指令经过去抖，避免键重复间隙误判造成「先停再走」的顿挫。

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



# 与同目录测试脚本一致，便于直接 python keyboard_remote_stdin.py

_car_dir = Path(__file__).resolve().parent

if str(_car_dir) not in sys.path:

    sys.path.insert(0, str(_car_dir))



from dual_motor_control import DifferentialDrive, create_dual_motor_driver



MOTION_KEYS = frozenset({"w", "s", "a", "d"})

# 某键最后一次在输入流中出现后，超过该时间则认为已松开（需大于按键首次重复的常见延时）

KEY_IDLE_SEC = 0.01

POLL_SEC = 0.01

# 连续判定为「无运动键」达该时间才真正停车，吃掉键重复的短暂空窗

STOP_DEBOUNCE_SEC = 0.02





def apply_motion(drive: DifferentialDrive, active: Set[str]) -> None:

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

    p = argparse.ArgumentParser(description="终端 WASD 双电机遥控")

    p.add_argument(

        "--speed",

        "-s",

        type=int,

        default=50,

        metavar="N",

        help="初始整体速度 0～100（%% 占空比基准），默认 50",

    )

    return p.parse_args(argv)





def main(argv: Optional[list[str]] = None) -> None:

    args = parse_args(argv)

    base_speed = max(0, min(100, args.speed))



    drive: Optional[DifferentialDrive] = None

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



    drive = create_dual_motor_driver(base_speed=base_speed)

    drive.base_speed = base_speed

    last_seen: dict[str, float] = {}

    prev_sigint = signal.signal(signal.SIGINT, on_signal)



    old_term = termios.tcgetattr(fd)

    tty.setcbreak(fd)



    verbose_diag = os.environ.get("KEYBOARD_REMOTE_DEBUG")



    if not sys.stdin.isatty():

        print(

            "[警告] stdin 不是交互式 TTY，键盘可能无效。请在本地终端前台运行（勿用无 PTY 的 ssh/任务）。",

            file=sys.stderr,

        )



    print(

        "终端键盘遥控 | W/S 前进/后退 | A/D 原地转 | W+A/W+D 前进左/右弯\n"

        "空格/x 急停 | +/- 调速(步进5%%) | [ / ] 调速(步进10%%) | 1～9→10～90%% 速 | 0→100%%速\n"

        f"松手判定: 键空闲≥{KEY_IDLE_SEC * 1000:.0f}ms 且无运动意图 ≥{STOP_DEBOUNCE_SEC * 1000:.0f}ms 才刹车\n"

        f"当前整体速度 base_speed={drive.base_speed}%% （可用 --speed 或 -s 指定启动速度）"

    )

    if verbose_diag:

        print(

            f"[KEYBOARD_REMOTE_DEBUG] stdin isatty={sys.stdin.isatty()} fd={fd} "

            "os.read 原始 fd"

        )



    last_cmd: Optional[tuple] = None

    stop_idle_since: Optional[float] = None



    try:

        while True:

            speed_dirty = False

            # 必须对「原始 fd」用 os.read；sys.stdin.read() 受 TextIOWrapper/行缓冲影响

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

            active = {k for k, t in last_seen.items() if k in MOTION_KEYS and (now - t) <= KEY_IDLE_SEC}

            sig = _compute_sig(active)



            # 尚有运动意图：立刻应用或与上次比较更新；并清除停车去抖时钟

            if sig != ("stop",):

                stop_idle_since = None

                # 调速后须重写 PWM（占空比随 base_speed）；换动作也须更新

                if speed_dirty or sig != last_cmd:

                    apply_motion(drive, active)

                    last_cmd = sig

                continue


            # 以下为「瞬时无运动意图」——可能仅是键重复的间隙；去抖后才真正停车

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

        restore_tty()





if __name__ == "__main__":

    try:

        main()

    except SystemExit:

        pass


