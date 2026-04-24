"""
三轴角度传感器读取示例代码

演示如何使用 TripleAxisReader 接口读取三轴传感器数据
"""

import sys
import os

# 添加上级目录到路径，以便导入 triple_axis_reader
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from triple_axis_reader import (
    axis,                    # 函数式调用
    axis_continuous,         # 连续读取
    axis_close,              # 关闭连接
    TripleAxisReader,        # 面向对象方式
    SensorMode               # 模式常量
)
import time


def example_basic():
    """基础用法：读取一次数据"""
    print("=" * 50)
    print("示例1: 基础读取")
    print("=" * 50)

    # 直接调用 axis 函数读取数据
    angles = axis(port='/dev/ttyS0', baudrate=115200, model=4)

    if angles:
        x, y, z = angles
        print(f"轴1 (X): {x:+.2f}°")
        print(f"轴2 (Y): {y:+.2f}°")
        print(f"轴3 (Z): {z:+.2f}°")
    else:
        print("读取失败，请检查串口连接")

    # 使用完毕后关闭
    axis_close()


def example_continuous():
    """连续读取多组数据"""
    print("\n" + "=" * 50)
    print("示例2: 连续读取10组数据")
    print("=" * 50)

    # 读取10组数据，间隔10ms
    data = axis_continuous(
        port='/dev/ttyS0',
        baudrate=115200,
        model=4,           # 100Hz模式
        count=10,          # 读取10组
        interval=0.01      # 间隔10ms
    )

    print(f"成功读取 {len(data)} 组数据:\n")
    for i, angles in enumerate(data, 1):
        print(f"第{i:2d}组: X={angles[0]:+7.2f}°  Y={angles[1]:+7.2f}°  Z={angles[2]:+7.2f}°")

    axis_close()


def example_oop_style():
    """面向对象方式"""
    print("\n" + "=" * 50)
    print("示例3: 面向对象方式")
    print("=" * 50)

    # 使用 with 语句自动管理资源
    with TripleAxisReader('/dev/ttyS0', baudrate=115200) as reader:
        # 设置为100Hz模式
        reader.set_mode(4)

        # 读取5组数据
        for i in range(5):
            angles = reader.read(timeout=0.2)
            if angles:
                print(f"X={angles[0]:+.2f}°  Y={angles[1]:+.2f}°  Z={angles[2]:+.2f}°")
            time.sleep(0.05)

    # with 块结束后自动关闭串口


def example_polling_mode():
    """问答模式读取"""
    print("\n" + "=" * 50)
    print("示例4: 问答模式(模式0)")
    print("=" * 50)

    with TripleAxisReader('/dev/ttyS0', baudrate=115200) as reader:
        reader.set_mode(SensorMode.POLLING)  # 或 mode=0

        for i in range(5):
            angles = reader.read(timeout=0.1)
            if angles:
                x, y, z = angles
                print(f"X={x:+.2f}°  Y={y:+.2f}°  Z={z:+.2f}°")
            time.sleep(0.1)


def example_history():
    """历史数据管理"""
    print("\n" + "=" * 50)
    print("示例5: 历史数据管理")
    print("=" * 50)

    with TripleAxisReader('/dev/ttyS0', baudrate=115200) as reader:
        reader.set_mode(4)

        # 连续读取
        reader.read_continuous(count=20, interval=0.01)

        # 获取所有历史数据
        history = reader.get_history()
        print(f"已保存 {len(history)} 组历史数据")

        # 获取最近5组
        recent = reader.get_history(count=5)
        print(f"\n最近5组数据:")
        for angles in recent:
            print(f"  X={angles[0]:+.2f}°  Y={angles[1]:+.2f}°  Z={angles[2]:+.2f}°")

        # 清除历史
        reader.clear_history()
        print(f"\n已清除历史，当前历史数据: {len(reader.get_history())} 组")


def example_mode_list():
    """模式列表"""
    print("\n" + "=" * 50)
    print("可用模式")
    print("=" * 50)

    for mode, (cmd, name) in SensorMode.MODES.items():
        print(f"  模式 {mode}: {name}")


def example_loop_read():
    """循环读取示例（模拟实时监控）"""
    print("\n" + "=" * 50)
    print("示例6: 循环读取（按 Ctrl+C 退出）")
    print("=" * 50)

    print("\n实时角度数据:")
    print("-" * 40)

    try:
        with TripleAxisReader('/dev/ttyS0', baudrate=115200) as reader:
            reader.set_mode(4)

            while True:
                angles = reader.read(timeout=0.1)
                if angles:
                    # 格式化输出
                    print(f"\rX={angles[0]:+7.2f}°  Y={angles[1]:+7.2f}°  Z={angles[2]:+7.2f}°", end="")
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n已停止读取")


if __name__ == "__main__":
    # 运行所有示例（按需取消注释）

    # 基础示例
    example_basic()

    # 连续读取
    example_continuous()

    # 面向对象方式
    example_oop_style()

    # 问答模式
    example_polling_mode()

    # 历史数据
    example_history()

    # 模式列表
    example_mode_list()

    # 循环读取（需要取消注释）
    # example_loop_read()

    print("\n" + "=" * 50)
    print("所有示例运行完成")
    print("=" * 50)
