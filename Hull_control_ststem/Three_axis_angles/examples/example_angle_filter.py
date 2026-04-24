"""
三轴角度滤波演示脚本

模拟 DDM350B 罗盘数据，演示角度滤波效果
"""

import math
import time
from angle_filter import (
    TripleAxisFilter,
    filter_angles,
    filter_yaw,
    yaw_to_display,
    FilteredAngleData,
    reset_filters
)


def simulate_jump_data():
    """
    模拟带有跳变的航向角数据

    场景：航向角从 350° 变化到 10°（跨越 0°）
    """
    print("=" * 60)
    print("测试1: 航向角 0°/360° 跳变处理")
    print("=" * 60)

    yaw_filter = TripleAxisFilter(yaw_coeff=0.2)

    # 模拟 350° -> 10° 的跳变
    raw_angles = [350, 352, 355, 358, 2, 5, 8, 10, 12]

    print(f"{'原始值':>10} | {'滤波前(-180~180)':>18} | {'滤波后(-180~180)':>18} | {'显示(0~360)':>12}")
    print("-" * 70)

    for raw_yaw in raw_angles:
        # 转换为有符号角度
        signed = raw_yaw - 360 if raw_yaw > 180 else raw_yaw
        # 滤波
        filtered_signed = yaw_filter.filter_yaw_only(raw_yaw)
        # 转回显示格式
        display = yaw_to_display(filtered_signed)

        print(f"{raw_yaw:>10.1f} | {signed:>18.1f} | {filtered_signed:>18.1f} | {display:>12.1f}")


def simulate_noise_data():
    """
    模拟带噪声的三轴角度数据
    """
    print("\n" + "=" * 60)
    print("测试2: 带噪声的三轴角度滤波")
    print("=" * 60)

    # 创建不同滤波系数的滤波器
    filters = {
        "平滑(0.1)": TripleAxisFilter(yaw_coeff=0.1, pitch_coeff=0.1, roll_coeff=0.1),
        "平衡(0.2)": TripleAxisFilter(yaw_coeff=0.2, pitch_coeff=0.2, roll_coeff=0.2),
        "灵敏(0.4)": TripleAxisFilter(yaw_coeff=0.4, pitch_coeff=0.4, roll_coeff=0.4),
    }

    # 模拟 100 个周期的数据
    print(f"\n{'周期':>5} | {'真值':>8} | {'平滑':>8} | {'平衡':>8} | {'灵敏':>8}")
    print("-" * 55)

    for i in range(20):
        true_yaw = 90.0  # 恒定航向
        noise = (math.sin(i * 0.5) * 5 + (hash(i) % 100 - 50) * 0.1)

        raw_yaw = true_yaw + noise

        print(f"{i:>5} | {true_yaw:>8.1f}", end="")

        for name, f in filters.items():
            filtered = f.filter_yaw_only(raw_yaw)
            print(f" | {filtered:>8.1f}", end="")
        print()


def continuous_mode_demo():
    """
    连续输出模式演示
    """
    print("\n" + "=" * 60)
    print("测试3: 连续输出模式")
    print("=" * 60)

    compass_filter = TripleAxisFilter(
        yaw_coeff=0.15,
        pitch_coeff=0.15,
        roll_coeff=0.15
    )

    # 模拟旋转 0° -> 360°
    print(f"\n{'#':>3} | {'Raw Roll':>10} | {'Raw Pitch':>10} | {'Raw Yaw':>10} | "
          f"{'Flt Roll':>10} | {'Flt Pitch':>10} | {'Flt Yaw':>10} | {'Display Yaw':>12}")
    print("-" * 95)

    for i in range(36):
        # 模拟缓慢旋转
        true_roll = 5 * math.sin(i * 0.3)
        true_pitch = 10 * math.cos(i * 0.2)
        true_yaw = (i * 10) % 360

        # 添加小噪声
        noise_roll = (hash(i * 3) % 100 - 50) * 0.05
        noise_pitch = (hash(i * 5) % 100 - 50) * 0.05
        noise_yaw = (hash(i * 7) % 100 - 50) * 0.1

        raw_roll = true_roll + noise_roll
        raw_pitch = true_pitch + noise_pitch
        raw_yaw = true_yaw + noise_yaw

        # 滤波
        flt_roll, flt_pitch, flt_yaw = compass_filter.filter(raw_roll, raw_pitch, raw_yaw)

        # 显示角度转回 0~360
        display_yaw = yaw_to_display(flt_yaw)

        print(f"{i:>3} | {raw_roll:>10.2f} | {raw_pitch:>10.2f} | {raw_yaw:>10.2f} | "
              f"{flt_roll:>10.2f} | {flt_pitch:>10.2f} | {flt_yaw:>10.2f} | {display_yaw:>12.2f}")


def edge_case_demo():
    """
    边界情况测试
    """
    print("\n" + "=" * 60)
    print("测试4: 边界情况处理")
    print("=" * 60)

    yaw_filter = CircularAngleFilter(filter_coeff=0.2)

    # 测试 -180° -> +180° 跳变
    print("\n测试 -180° -> +180° 跳变:")
    test_values = [-175, -178, -180, 178, 175, 172]

    print(f"{'原始值':>10} | {'有符号':>10} | {'差角':>10} | {'滤波后':>10}")
    print("-" * 50)

    for raw in test_values:
        signed = raw - 360 if raw > 180 else raw
        filtered = yaw_filter.filter(raw)
        print(f"{raw:>10.1f} | {signed:>10.1f} | {filtered - signed:>10.1f} | {filtered:>10.1f}")


def usage_demo():
    """
    使用方式演示
    """
    print("\n" + "=" * 60)
    print("使用方式演示")
    print("=" * 60)

    print("""
    # 方式1: 快捷函数（单次调用）
    from angle_filter import filter_angles, yaw_to_display

    roll, pitch, yaw_signed = filter_angles(raw_roll, raw_pitch, raw_yaw)
    yaw_display = yaw_to_display(yaw_signed)  # 转回 0~360

    # 方式2: 扩展函数（返回完整数据）
    from angle_filter import filter_angles_ex

    result = filter_angles_ex(raw_roll, raw_pitch, raw_yaw)
    print(result.roll, result.pitch, result.yaw, result.yaw_signed)

    # 方式3: 单独使用环形角度滤波器
    from angle_filter import CircularAngleFilter

    yaw_filter = CircularAngleFilter(filter_coeff=0.15)
    for raw_yaw in yaw_data_stream:
        filtered_yaw = yaw_filter.filter(raw_yaw)  # 返回 -180~+180
        display_yaw = yaw_to_display(filtered_yaw)  # 转为 0~360

    # 方式4: 与 DDM350B 集成
    from angle_filter import process_ddm350b_data

    compass = DDM350B('/dev/ttyS0')
    compass.connect()
    data = compass.read()
    filtered = process_ddm350b_data(data)
    print(f"Yaw: {filtered.yaw:.1f}°  Pitch: {filtered.pitch:.1f}°  Roll: {filtered.roll:.1f}°")
    """)


if __name__ == "__main__":
    print("三轴角度滤波演示")
    print("=" * 60)

    simulate_jump_data()
    simulate_noise_data()
    continuous_mode_demo()
    edge_case_demo()
    usage_demo()

    print("\n演示完成!")