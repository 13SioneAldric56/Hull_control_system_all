"""
电调（ESC）无刷电机控制模块 — GPIO 软件 PWM（sysfs）
适用于 APISQUEEN 等标准 50Hz 舵机协议电调

引脚: GPIO54 = GPIO2_C3

说明：
  - 不使用 pwmchip4，直接 export GPIO54 输出 50Hz 脉宽调制
  - 文档「占空比」= range1000 档位，实际脉宽 = 档位/1000 × 20ms
  - 中位 75 = 1.5ms（解锁/停转）
"""
import threading
import time

# ===================== 引脚定义 =====================
ESC_MOTOR = {
    "pwm_gpio_num": 54,     # GPIO2_C3
}

PWM_CHIP_RELEASE = "pwmchip4"   # 同脚硬件 PWM，初始化前须释放
PWM_DEV_RELEASE = "pwm0"

PWM_FREQUENCY = 50          # Hz，周期 20ms
PERIOD_US = 20_000
PWM_RANGE = 1000

PULSE_STOP = 75             # 1.5ms 停转/解锁
PULSE_FORWARD_MIN = 75
PULSE_FORWARD_MAX = 100     # 2.0ms 正转最快
PULSE_REVERSE_MIN = 50      # 1.0ms 反转最快
PULSE_REVERSE_MAX = 75
UNLOCK_HOLD_SEC = 3.0


def pulse_to_us(pulse_value: int) -> int:
    pulse_value = max(0, min(PWM_RANGE, int(pulse_value)))
    return int(PERIOD_US * pulse_value / PWM_RANGE)


def pulse_to_ms(pulse_value: int) -> float:
    return pulse_to_us(pulse_value) / 1000.0


def pulse_to_duty_percent(pulse_value: int) -> float:
    """相对 20ms 周期的高电平占空比（%），供示波器对照。"""
    return pulse_value / PWM_RANGE * 100.0


def _delay_us(us: float) -> None:
    if us <= 0:
        return
    end = time.perf_counter() + us / 1_000_000
    while time.perf_counter() < end:
        pass


def release_pwmchip(pwm_chip: str = PWM_CHIP_RELEASE, pwm_dev: str = PWM_DEV_RELEASE) -> None:
    """关闭并释放同脚硬件 PWM，避免与 GPIO 冲突。"""
    enable_path = f"/sys/class/pwm/{pwm_chip}/{pwm_dev}/enable"
    try:
        with open(enable_path, "w") as f:
            f.write("0")
    except OSError:
        pass
    try:
        with open(f"/sys/class/pwm/{pwm_chip}/unexport", "w") as f:
            f.write("0\n" if "pwm0" in pwm_dev else "1\n")
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


class GpioSoftPwm:
    """GPIO sysfs 软件 PWM（50Hz 舵机/电调协议）"""

    def __init__(self, pin: int, frequency: int = PWM_FREQUENCY):
        self.pin = pin
        self.period_us = int(1_000_000 / frequency)
        self._pulse_us = pulse_to_us(PULSE_STOP)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._value_file = None

    def init(self) -> None:
        release_pwmchip()
        gpio_export(self.pin)
        gpio_set_output(self.pin)
        self._stop.clear()
        value_path = f"/sys/class/gpio/gpio{self.pin}/value"
        self._value_file = open(value_path, "w")
        self._thread = threading.Thread(target=self._pwm_loop, daemon=True)
        self._thread.start()

    def set_pulse(self, pulse_value: int) -> None:
        with self._lock:
            self._pulse_us = pulse_to_us(pulse_value)

    def _pwm_loop(self) -> None:
        vf = self._value_file
        while not self._stop.is_set():
            with self._lock:
                high_us = self._pulse_us
            low_us = max(0, self.period_us - high_us)

            vf.write("1")
            vf.flush()
            _delay_us(high_us)

            if self._stop.is_set():
                break

            vf.write("0")
            vf.flush()
            _delay_us(low_us)

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._value_file:
            try:
                self._value_file.write("0")
                self._value_file.flush()
                self._value_file.close()
            except OSError:
                pass
            self._value_file = None
        self._thread = None


class EscMotor:
    """电调控制（GPIO 软件 PWM）"""

    def __init__(self, config: dict = None):
        cfg = config or ESC_MOTOR
        self.pwm_gpio_num = cfg["pwm_gpio_num"]
        self._pwm = GpioSoftPwm(self.pwm_gpio_num)
        self._initialized = False
        self._unlocked = False

    def init(self) -> None:
        if self._initialized:
            return
        self._pwm.init()
        self._pwm.set_pulse(PULSE_STOP)
        self._initialized = True
        print(
            f"[EscMotor] GPIO{self.pwm_gpio_num} "
            f"软件PWM {PWM_FREQUENCY}Hz"
        )

    def set_pulse(self, pulse_value: int) -> None:
        self.init()
        pulse_value = max(0, min(PWM_RANGE, int(pulse_value)))
        self._pwm.set_pulse(pulse_value)
        print(
            f"[EscMotor] 档位={pulse_value} "
            f"脉宽={pulse_to_ms(pulse_value):.3f}ms "
            f"占空比={pulse_to_duty_percent(pulse_value):.1f}%"
        )

    def unlock(self, hold_sec: float = UNLOCK_HOLD_SEC) -> None:
        print(f"[EscMotor] 解锁，中位 {hold_sec}s ...")
        self.set_pulse(PULSE_STOP)
        time.sleep(hold_sec)
        self._unlocked = True
        print("[EscMotor] 解锁完成")

    def stop(self) -> None:
        if self._initialized:
            self.set_pulse(PULSE_STOP)

    def forward(self, speed: int = 50) -> None:
        speed = max(0, min(100, speed))
        pulse = PULSE_FORWARD_MIN + int(
            speed * (PULSE_FORWARD_MAX - PULSE_FORWARD_MIN) / 100
        )
        self.set_pulse(pulse)

    def reverse(self, speed: int = 50) -> None:
        speed = max(0, min(100, speed))
        pulse = PULSE_REVERSE_MAX - int(
            speed * (PULSE_REVERSE_MAX - PULSE_REVERSE_MIN) / 100
        )
        self.set_pulse(pulse)

    def shutdown(self) -> None:
        if not self._initialized:
            return
        self._pwm.set_pulse(PULSE_STOP)
        time.sleep(0.05)
        self._pwm.stop()
        gpio_unexport(self.pwm_gpio_num)
        self._initialized = False
        print("[EscMotor] GPIO PWM 已关闭")


def create_esc_motor(config: dict = None, auto_unlock: bool = True) -> EscMotor:
    motor = EscMotor(config)
    motor.init()
    if auto_unlock:
        motor.unlock()
    return motor
