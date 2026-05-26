#!/usr/bin/env python3
"""
硬件 PWM：pwmchip1/pwm0（需 overlay pwm3-m2）
50Hz：先 7.5% 占空比 5 秒，再 8% 占空比 5 秒

引脚: GPIO50 = GPIO1_D2 = PWM3_M2 = pwmchip1/pwm0

用法: sudo python3 gpio50_pwm.py
"""
import signal
import time

GPIO_PIN = 50
PWM_CHIP = "pwmchip1"
PWM_DEV = "pwm0"

PERIOD_NS = 20_000_000   # 20 ms，50Hz
HOLD_SEC = 5

_running = True


def _stop(*_):
    global _running
    _running = False


def _pwm_path(attr: str) -> str:
    return f"/sys/class/pwm/{PWM_CHIP}/{PWM_DEV}/{attr}"


def _write(path: str, val: str) -> None:
    with open(path, "w") as f:
        f.write(val)


def duty_ns(percent: float) -> int:
    return int(PERIOD_NS * percent / 100.0)


def set_duty_percent(percent: float) -> None:
    ns = duty_ns(percent)
    _write(_pwm_path("duty_cycle"), str(ns))
    print(f"占空比 {percent}% -> duty_cycle={ns} ns ({ns / 1_000_000:.3f} ms)")


def release_gpio() -> None:
    try:
        with open("/sys/class/gpio/unexport", "w") as f:
            f.write(f"{GPIO_PIN}\n")
    except OSError:
        pass


def pwm_setup(initial_percent: float = 7.5) -> None:
    release_gpio()
    try:
        with open(f"/sys/class/pwm/{PWM_CHIP}/unexport", "w") as f:
            f.write("0\n")
        time.sleep(0.2)
    except OSError:
        pass
    try:
        with open(f"/sys/class/pwm/{PWM_CHIP}/export", "w") as f:
            f.write("0\n")
        time.sleep(0.2)
    except OSError:
        pass

    _write(_pwm_path("period"), str(PERIOD_NS))
    set_duty_percent(initial_percent)
    try:
        _write(_pwm_path("polarity"), "normal")
    except OSError:
        pass
    _write(_pwm_path("enable"), "1")


def pwm_shutdown() -> None:
    try:
        _write(_pwm_path("enable"), "0")
    except OSError:
        pass
    try:
        with open(f"/sys/class/pwm/{PWM_CHIP}/unexport", "w") as f:
            f.write("0\n")
    except OSError:
        pass


def _sleep(seconds: float) -> None:
    end = time.monotonic() + seconds
    while _running and time.monotonic() < end:
        time.sleep(0.1)


def main() -> None:
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    pwm_setup(7.5)
    print(f"GPIO{GPIO_PIN} <- {PWM_CHIP}/{PWM_DEV}, 50Hz period={PERIOD_NS} ns")

    try:
        print(f"阶段1: 7.5% 持续 {HOLD_SEC}s")
        _sleep(HOLD_SEC)
        if not _running:
            return

        set_duty_percent(8.0)
        print(f"阶段2: 8% 持续 {HOLD_SEC}s")
        _sleep(HOLD_SEC)
    finally:
        pwm_shutdown()
        print("PWM 已关闭")


if __name__ == "__main__":
    main()
