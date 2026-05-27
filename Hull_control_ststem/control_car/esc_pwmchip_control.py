"""
双路电调（ESC）— sysfs pwmchip 硬件 PWM，50Hz 舵机协议

左: pwmchip2/pwm0 (GPIO50)  右: pwmchip0/pwm0 (GPIO58)
档位与脉宽语义同 esc_motor_control.py（75=7.5% 中位停转/解锁）
"""
import time

try:
    from control_car.dual_motor_control import LEFT_MOTOR, RIGHT_MOTOR
    from control_car.esc_motor_control import (
        PERIOD_US,
        PULSE_FORWARD_MAX,
        PULSE_FORWARD_MIN,
        PULSE_REVERSE_MAX,
        PULSE_REVERSE_MIN,
        PULSE_STOP,
        PWM_RANGE,
        UNLOCK_HOLD_SEC,
        pulse_to_duty_percent,
        pulse_to_ms,
    )
    from control_car.pwm_sysfs import pwm_release, pwm_setup, pwm_set_duty_ns
except ImportError:
    from dual_motor_control import LEFT_MOTOR, RIGHT_MOTOR
    from esc_motor_control import (
        PERIOD_US,
        PULSE_FORWARD_MAX,
        PULSE_FORWARD_MIN,
        PULSE_REVERSE_MAX,
        PULSE_REVERSE_MIN,
        PULSE_STOP,
        PWM_RANGE,
        UNLOCK_HOLD_SEC,
        pulse_to_duty_percent,
        pulse_to_ms,
    )
    from pwm_sysfs import pwm_release, pwm_setup, pwm_set_duty_ns

PERIOD_NS = PERIOD_US * 1000  # 20_000_000 ns，50Hz


def pulse_to_duty_ns(pulse_value: int) -> int:
    pulse_value = max(0, min(PWM_RANGE, int(pulse_value)))
    return int(PERIOD_NS * pulse_value / PWM_RANGE)


class PwmchipEscMotor:
    """单路电调：pwmchip sysfs 硬件 PWM"""

    def __init__(self, config: dict, label: str = ""):
        self.pwm_gpio_num = config["pwm_gpio_num"]
        self.pwm_chip = config["pwm_chip"]
        self.pwm_dev = config["pwm_dev"]
        self.label = label or self.pwm_chip
        self._initialized = False
        self._unlocked = False
        self._pulse = PULSE_STOP

    def init(self) -> None:
        if self._initialized:
            return
        duty_ns = pulse_to_duty_ns(PULSE_STOP)
        pwm_setup(
            self.pwm_chip,
            self.pwm_dev,
            PERIOD_NS,
            duty_ns=duty_ns,
            gpio_num=self.pwm_gpio_num,
            enable=True,
        )
        self._initialized = True
        print(
            f"[{self.label}] {self.pwm_chip}/{self.pwm_dev} GPIO{self.pwm_gpio_num} "
            f"50Hz period={PERIOD_NS} 中位={pulse_to_duty_percent(PULSE_STOP):.1f}%"
        )

    def _apply_pulse(self, pulse_value: int, log: bool = True) -> None:
        pulse_value = max(0, min(PWM_RANGE, int(pulse_value)))
        self._pulse = pulse_value
        if self._initialized:
            pwm_set_duty_ns(
                self.pwm_chip,
                self.pwm_dev,
                pulse_to_duty_ns(pulse_value),
                PERIOD_NS,
            )
        if log:
            print(
                f"[{self.label}] 档位={pulse_value} "
                f"脉宽={pulse_to_ms(pulse_value):.3f}ms "
                f"占空比={pulse_to_duty_percent(pulse_value):.1f}%"
            )

    def set_pulse(self, pulse_value: int) -> None:
        self.init()
        self._apply_pulse(pulse_value)

    def unlock(self, hold_sec: float = UNLOCK_HOLD_SEC) -> None:
        self.init()
        print(f"[{self.label}] 解锁，中位 {hold_sec}s ...")
        self._apply_pulse(PULSE_STOP, log=False)
        time.sleep(hold_sec)
        self._unlocked = True
        print(f"[{self.label}] 解锁完成")

    def stop(self) -> None:
        if self._initialized:
            self._apply_pulse(PULSE_STOP, log=False)

    def forward(self, speed: int = 50) -> None:
        self.init()
        speed = max(0, min(100, speed))
        pulse = PULSE_FORWARD_MIN + int(
            speed * (PULSE_FORWARD_MAX - PULSE_FORWARD_MIN) / 100
        )
        self._apply_pulse(pulse, log=False)

    def reverse(self, speed: int = 50) -> None:
        self.init()
        speed = max(0, min(100, speed))
        pulse = PULSE_REVERSE_MAX - int(
            speed * (PULSE_REVERSE_MAX - PULSE_REVERSE_MIN) / 100
        )
        self._apply_pulse(pulse, log=False)

    def shutdown(self) -> None:
        if not self._initialized:
            return
        self.stop()
        time.sleep(0.05)
        pwm_release(self.pwm_chip, self.pwm_dev)
        self._initialized = False
        print(f"[{self.label}] PWM 已关闭 (unexport)")


def unlock_dual_esc(
    left: PwmchipEscMotor,
    right: PwmchipEscMotor,
    hold_sec: float = UNLOCK_HOLD_SEC,
) -> None:
    """两路同时输出 7.5% 中位并保持 hold_sec 秒（电调解锁）。"""
    left.init()
    right.init()
    print(f"[ESC] 双路同时解锁，中位 {hold_sec}s ...")
    left._apply_pulse(PULSE_STOP, log=False)
    right._apply_pulse(PULSE_STOP, log=False)
    time.sleep(hold_sec)
    left._unlocked = True
    right._unlocked = True
    print("[ESC] 双路解锁完成")


def create_left_esc() -> PwmchipEscMotor:
    return PwmchipEscMotor(LEFT_MOTOR, label="左电调")


def create_right_esc() -> PwmchipEscMotor:
    return PwmchipEscMotor(RIGHT_MOTOR, label="右电调")
