"""
双电机差速驱动测试脚本
测试直行、停止、原地旋转、差速转弯等功能
"""
import sys
import time

# 添加当前目录以便导入模块
sys.path.insert(0, __file__.rsplit('/', 1)[0] if '/' in __file__ else '.')

from dual_motor_control import (
    create_dual_motor_driver,
    DifferentialDrive
)


def wait(seconds: float, description: str = ""):
    """等待并打印提示"""
    if description:
        print(f"  └─ {description}")
    time.sleep(seconds)


def test_basic_movement(driver: DifferentialDrive):
    """测试基础运动：直行和停止"""
    print("\n" + "=" * 60)
    print("【测试1】基础运动测试")
    print("=" * 60)

    # 测试直行
    print("\n▶ 测试直行...")
    driver.forward(40)
    wait(2, "直行 40% 速度，持续 2 秒")

    # 加速
    print("\n▶ 测试加速...")
    driver.forward(70)
    wait(2, "加速到 70% 速度，持续 2 秒")

    # 刹车停止
    print("\n▶ 测试刹车停止...")
    driver.stop(brake=True)
    wait(1, "刹车急停，持续 1 秒")

    # 滑行停止
    print("\n▶ 测试滑行停止...")
    driver.forward(60)
    wait(1.5, "直行 60% 速度")
    driver.stop(brake=False)
    wait(1, "滑行缓停，持续 1 秒")

    print("\n✅ 基础运动测试完成")


def test_backward(driver: DifferentialDrive):
    """测试后退"""
    print("\n" + "=" * 60)
    print("【测试2】后退测试")
    print("=" * 60)

    print("\n▶ 后退行驶...")
    driver.backward(30)
    wait(2, "后退 30% 速度，持续 2 秒")

    driver.stop()
    wait(0.5)

    print("\n✅ 后退测试完成")


def test_spin_turns(driver: DifferentialDrive):
    """测试原地旋转"""
    print("\n" + "=" * 60)
    print("【测试3】原地旋转测试")
    print("=" * 60)

    # 原地左转
    print("\n▶ 原地左旋转...")
    driver.spin_left(50)
    wait(2, "原地左转 50% 速度，持续 2 秒")
    driver.stop()
    wait(0.5)

    # 原地右转
    print("\n▶ 原地右旋转...")
    driver.spin_right(50)
    wait(2, "原地右转 50% 速度，持续 2 秒")
    driver.stop()
    wait(0.5)

    print("\n✅ 原地旋转测试完成")


def test_differential_turns(driver: DifferentialDrive):
    """测试差速转弯"""
    print("\n" + "=" * 60)
    print("【测试4】差速转弯测试")
    print("=" * 60)

    # 打印所有可用转弯配置
    profiles = driver.get_turn_profiles()
    print(f"\n可用的转弯配置 ({len(profiles)} 种):")
    for i, name in enumerate(profiles, 1):
        print(f"  {i:2}. {name}")

    # 逐一测试各类转弯
    test_cases = [
        ("差速左微调", "轻微左转修正"),
        ("差速右微调", "轻微右转修正"),
        ("差速左小弯", "小角度左转"),
        ("差速右小弯", "小角度右转"),
        ("差速左中弯", "中角度左转"),
        ("差速右中弯", "中角度右转"),
        ("差速左大弯", "大角度左转"),
        ("差速右大弯", "大角度右转"),
        ("差速左急弯", "急弯左转"),
        ("差速右急弯", "急弯右转"),
    ]

    print("\n▶ 逐一测试差速转弯...")
    for profile_name, description in test_cases:
        print(f"\n  {profile_name}: {description}")
        driver.differential_turn(profile_name, 60)
        wait(1.5, f"持续 1.5 秒")
        driver.stop()
        wait(0.3)

    print("\n✅ 差速转弯测试完成")


def test_spin_variants(driver: DifferentialDrive):
    """测试原地旋转变体"""
    print("\n" + "=" * 60)
    print("【测试5】原地旋转变体测试")
    print("=" * 60)

    spin_tests = [
        ("原地左小转", "原地小幅度左转"),
        ("原地右小转", "原地小幅度右转"),
        ("原地左中转", "原地中等幅度左转"),
        ("原地右中转", "原地中等幅度右转"),
        ("原地左急转", "原地急速左转"),
        ("原地右急转", "原地急速右转"),
    ]

    for profile_name, description in spin_tests:
        print(f"\n  {profile_name}: {description}")
        driver.differential_turn(profile_name, 60)
        wait(1.5, f"持续 1.5 秒")
        driver.stop()
        wait(0.3)

    print("\n✅ 原地旋转变体测试完成")


def test_custom_turn(driver: DifferentialDrive):
    """测试自定义差速"""
    print("\n" + "=" * 60)
    print("【测试6】自定义差速测试")
    print("=" * 60)

    # 自定义差速测试用例
    custom_tests = [
        # (左轮系数, 右轮系数, 描述)
        (0.0, 1.0, "左轮停 右轮全速（向左原地旋转准备）"),
        (1.0, 0.0, "左轮全速 右轮停（向右原地旋转准备）"),
        (0.5, 0.8, "左轮50% 右轮80%（微右转）"),
        (0.8, 0.5, "左轮80% 右轮50%（微左转）"),
        (-0.3, 0.6, "左轮反转30% 右轮60%（左急弯）"),
        (0.6, -0.3, "左轮60% 右轮反转30%（右急弯）"),
    ]

    print("\n▶ 测试自定义差速参数...")
    for left_f, right_f, description in custom_tests:
        print(f"\n  自定义: {description}")
        driver.custom_turn(left_f, right_f, 60)
        wait(1.5, "持续 1.5 秒")
        driver.stop()
        wait(0.3)

    print("\n✅ 自定义差速测试完成")


def test_comprehensive_movement(driver: DifferentialDrive):
    """综合运动测试：模拟实际行驶路线"""
    print("\n" + "=" * 60)
    print("【测试7】综合运动测试（模拟行驶路线）")
    print("=" * 60)

    route = [
        ("forward", 60, 2, "直行 2 秒"),
        ("turn_left", 50, 1.5, "左转 1.5 秒"),
        ("forward", 60, 3, "直行 3 秒"),
        ("turn_right", 50, 1.5, "右转 1.5 秒"),
        ("forward", 60, 2, "直行 2 秒"),
        ("spin_left", 50, 1, "原地左转 1 秒"),
        ("forward", 60, 2, "直行 2 秒"),
        ("spin_right", 50, 1, "原地右转 1 秒"),
        ("stop", None, 1, "停止 1 秒"),
    ]

    for action, value, duration, description in route:
        print(f"\n▶ {description}")
        if action == "forward":
            driver.forward(value)
        elif action == "backward":
            driver.backward(value)
        elif action == "turn_left":
            driver.turn_left(value)
        elif action == "turn_right":
            driver.turn_right(value)
        elif action == "spin_left":
            driver.spin_left(value)
        elif action == "spin_right":
            driver.spin_right(value)
        elif action == "stop":
            driver.stop(brake=True)
        wait(duration)
        driver.stop()

    print("\n✅ 综合运动测试完成")


def print_header():
    """打印测试标题"""
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║" + "  双电机差速驱动测试程序".center(56) + "║")
    print("║" + "  Dual Motor Differential Drive Test".center(56) + "║")
    print("╚" + "═" * 58 + "╝")
    print()


def print_turn_profile_guide(driver: DifferentialDrive):
    """打印转弯配置参考指南"""
    print("\n" + "=" * 60)
    print("【转弯配置参考指南】")
    print("=" * 60)

    categories = {
        "原地旋转": ["spin_left", "spin_right", "原地左小转", "原地右小转",
                    "原地左中转", "原地右中转", "原地左急转", "原地右急转"],
        "差速微调": ["差速左微调", "差速右微调"],
        "差速小弯": ["差速左小弯", "差速右小弯"],
        "差速中弯": ["差速左中弯", "差速右中弯"],
        "差速大弯": ["差速左大弯", "差速右大弯"],
        "差速急弯": ["差速左急弯", "差速右急弯"],
    }

    profiles = driver.TURN_PROFILES

    for category, names in categories.items():
        print(f"\n  【{category}】")
        for name in names:
            if name in profiles:
                left_f, right_f = profiles[name]
                print(f"    {name:12s}: 左轮 {'+' if left_f >= 0 else '-'}{abs(left_f):.1f}  右轮 {'+' if right_f >= 0 else '-'}{abs(right_f):.1f}")


def main():
    """主测试函数"""
    print_header()

    print("正在初始化双电机驱动...")
    try:
        driver = create_dual_motor_driver(base_speed=50)
    except Exception as e:
        print(f"\n❌ 初始化失败: {e}")
        print("请检查引脚配置是否正确")
        return

    # 打印转弯配置指南
    print_turn_profile_guide(driver)

    try:
        # 逐一运行测试
        test_basic_movement(driver)
        test_backward(driver)
        test_spin_turns(driver)
        test_differential_turns(driver)
        test_spin_variants(driver)
        test_custom_turn(driver)
        test_comprehensive_movement(driver)

        # 最终停止
        driver.stop()
        print("\n")
        print("╔" + "═" * 58 + "╗")
        print("║" + "  ✅ 全部测试完成！".center(56) + "║")
        print("╚" + "═" * 58 + "╝")
        print("\n")

    except KeyboardInterrupt:
        print("\n\n⚠ 检测到 Ctrl+C，正在安全停止...")
        driver.stop()
        print("电机已安全停止")


if __name__ == "__main__":
    main()
