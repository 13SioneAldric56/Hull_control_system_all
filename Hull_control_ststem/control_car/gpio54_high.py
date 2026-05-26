"""
GPIO54 输出高电平（sysfs）

引脚: GPIO54 = GPIO2_C3（与 pwmchip4 电调/左轮 PWM 同脚）
用法: sudo python3 gpio54_high.py
      sudo python3 gpio54_high.py --low   # 拉低
      sudo python3 gpio54_high.py --off   # 释放 export

注意: 若 pwmchip4 已占用该脚，须先关闭 PWM，否则 GPIO 可能无效。
"""
import argparse
import time

GPIO_PIN = 54
PWM_CHIP = "pwmchip4"
PWM_DEV = "pwm0"


def pwm_disable_if_present() -> None:
    """同脚 PWM 关闭，避免与 GPIO 功能冲突。"""
    enable_path = f"/sys/class/pwm/{PWM_CHIP}/{PWM_DEV}/enable"
    try:
        with open(enable_path, "w") as f:
            f.write("0")
        print(f"[gpio54] 已关闭 {PWM_CHIP}/{PWM_DEV} PWM")
    except OSError:
        pass


def gpio_export(pin: int) -> None:
    try:
        with open("/sys/class/gpio/export", "w") as f:
            f.write(f"{pin}\n")
        time.sleep(0.1)
    except OSError:
        pass


def gpio_unexport(pin: int) -> None:
    try:
        with open("/sys/class/gpio/unexport", "w") as f:
            f.write(f"{pin}\n")
    except OSError:
        pass


def gpio_set_output(pin: int) -> None:
    with open(f"/sys/class/gpio/gpio{pin}/direction", "w") as f:
        f.write("out")


def gpio_write(pin: int, level: int) -> None:
    with open(f"/sys/class/gpio/gpio{pin}/value", "w") as f:
        f.write("1" if level else "0")


def set_gpio54_high() -> None:
    pwm_disable_if_present()
    gpio_export(GPIO_PIN)
    gpio_set_output(GPIO_PIN)
    gpio_write(GPIO_PIN, 1)
    print(f"[gpio54] GPIO{GPIO_PIN} 已设为输出，高电平 (value=1)")


def set_gpio54_low() -> None:
    pwm_disable_if_present()
    gpio_export(GPIO_PIN)
    gpio_set_output(GPIO_PIN)
    gpio_write(GPIO_PIN, 0)
    print(f"[gpio54] GPIO{GPIO_PIN} 已设为输出，低电平 (value=0)")


def main() -> None:
    parser = argparse.ArgumentParser(description="GPIO54 电平控制")
    parser.add_argument("--low", action="store_true", help="输出低电平")
    parser.add_argument("--off", action="store_true", help="unexport 释放引脚")
    args = parser.parse_args()

    if args.off:
        gpio_unexport(GPIO_PIN)
        print(f"[gpio54] GPIO{GPIO_PIN} 已 unexport")
        return
    if args.low:
        set_gpio54_low()
    else:
        set_gpio54_high()


if __name__ == "__main__":
    main()
