"""
电调（ESC）测试 — GPIO54 软件 PWM（不使用 pwmchip4）
"""
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0] if "/" in __file__ else ".")

from esc_motor_control import (
    PULSE_FORWARD_MAX,
    PULSE_STOP,
    EscMotor,
    create_esc_motor,
)


def wait(seconds: float, description: str = "") -> None:
    if description:
        print(f"  └─ {description}")
    time.sleep(seconds)


def print_header() -> None:
    print()
    print("╔" + "═" * 58 + "╗")
    print("║" + "  电调 ESC 测试 (GPIO54 软件PWM)".center(54) + "║")
    print("║" + "  GPIO54 | 50Hz | 75=停转 | 100=正转 | 50~75=反转".center(54) + "║")
    print("╚" + "═" * 58 + "╝")
    print()


def test_scope_visible(motor: EscMotor) -> None:
    print("\n" + "=" * 60)
    print("【测试】示波器脉宽对比（每档 3s）")
    print("=" * 60)
    for pulse, desc in [(50, "1.0ms"), (75, "1.5ms"), (100, "2.0ms")]:
        print(f"\n▶ {desc} 档位={pulse}")
        motor.set_pulse(pulse)
        wait(3)
    motor.stop()
    print("\n✅ 示波器对比完成")


def test_document_flow(motor: EscMotor) -> None:
    print("\n" + "=" * 60)
    print("【测试】文档标准流程")
    print("=" * 60)

    print("\n▶ 解锁 (档位 75, 3s)")
    motor.unlock(hold_sec=999.0)

    print("\n▶ 正转 (档位 100, 15s)")
    motor.set_pulse(PULSE_FORWARD_MAX)
    wait(5, "正转 15s")

    #print("\n▶ 反转 (档位 60, 5s)")
    #motor.set_pulse(60)
    wait(5, "反转 5s")

    print("\n▶ 停转 (档位 75, 5s)")
    motor.set_pulse(PULSE_STOP)
    wait(5, "停转 5s")

    print("\n✅ 文档流程完成")


def main() -> None:
    print_header()
    print("初始化 GPIO54 软件 PWM...")
    print("⚠ 测试前请拆除螺旋桨")
    print("⚠ 需 root 权限: sudo python3 test_esc_motor.py")
    time.sleep(2)

    motor = None
    try:
        motor = create_esc_motor(auto_unlock=False)
        #motor.unlock()

        #test_scope_visible(motor)
        test_document_flow(motor)

        motor.stop()
        print()
        print("╔" + "═" * 58 + "╗")
        print("║" + "  ✅ 全部测试完成！".center(56) + "║")
        print("╚" + "═" * 58 + "╝")

    except KeyboardInterrupt:
        print("\n⚠ Ctrl+C，安全停转...")
        if motor:
            motor.stop()
    except Exception as e:
        print(f"\n❌ 失败: {e}")
        print("请检查: sudo、GPIO54 接线、是否被其他程序占用")
        if motor:
            motor.stop()
        raise
    finally:
        if motor:
            motor.shutdown()


if __name__ == "__main__":
    main()
