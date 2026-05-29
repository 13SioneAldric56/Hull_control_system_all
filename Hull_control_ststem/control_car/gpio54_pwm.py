#!/usr/bin/env python3
"""
GPIO54 软件 PWM：50Hz，脉宽 1.5ms（占空比 7.5%）
用法: sudo python3 gpio54_pwm_7p5.py
      Ctrl+C 退出
"""
import signal
import sys
import time

GPIO_PIN = 50
FREQ_HZ = 50
PERIOD_US = 1_000_000 // FREQ_HZ      # 20000 us
PULSE_US = 1200                        # 1.5 ms
LOW_US = PERIOD_US - PULSE_US          # 18500 us

_running = True

def _stop(*_):
    global _running
    _running = False

def gpio_export(pin: int) -> None:
    try:
        with open("/sys/class/gpio/export", "w") as f:
            f.write(f"{pin}\n")
        time.sleep(0.1)
    except OSError:
        pass

def gpio_set_output(pin: int) -> None:
    with open(f"/sys/class/gpio/gpio{pin}/direction", "w") as f:
        f.write("out")

def delay_us(us: float) -> None:
    end = time.perf_counter() + us / 1_000_000
    while time.perf_counter() < end:
        pass

def main() -> None:
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    gpio_export(GPIO_PIN)
    gpio_set_output(GPIO_PIN)
    value_path = f"/sys/class/gpio/gpio{GPIO_PIN}/value"

    print(f"GPIO{GPIO_PIN}: {FREQ_HZ}Hz, 高电平 {PULSE_US}us (7.5%), Ctrl+C 停止")

    with open(value_path, "w") as vf:
        while _running:
            t0 = time.perf_counter()
            vf.write("1")
            vf.flush()
            elapsed = (time.perf_counter() - t0) * 1_000_000
            delay_us(max(0, PULSE_US - elapsed))

            t0 = time.perf_counter()
            vf.write("0")
            vf.flush()
            elapsed = (time.perf_counter() - t0) * 1_000_000
            delay_us(max(0, LOW_US - elapsed))

        vf.write("0")
        vf.flush()

    print("已停止，输出低电平")

if __name__ == "__main__":
    main()