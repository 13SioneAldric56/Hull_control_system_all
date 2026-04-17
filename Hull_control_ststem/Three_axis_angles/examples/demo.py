#!/usr/bin/env python3
"""
DDM350B/DDM360B 三维电子罗盘 - 实用示例程序

运行前请确保：
1. 已安装 pyserial: pip install pyserial
2. 已正确连接罗盘设备
3. 确认串口号（可通过设备管理器查看）
"""

import sys
from pathlib import Path

# 添加父目录到路径，确保能导入 ddm350b 模块
sys.path.insert(0, str(Path(__file__).parent.parent))

from ddm350b import DDM350B, OutputMode, CompassData
import time


def get_com_port():
    """获取串口号（可在运行前修改）"""
    return 'COM3'


def example_1_polling_mode():
    """
    示例1: 问答模式读取
    - 需要主动发送读取命令
    - 适合低频率读取（<1Hz）
    """
    print("=" * 55)
    print("示例1: 问答模式（需要主动请求）")
    print("=" * 55)

    port = get_com_port()
    compass = DDM350B(port, baudrate=115200)

    if not compass.connect():
        print(f"[错误] 无法打开串口 {port}")
        return

    print(f"[信息] 已连接到 {port}")
    compass.set_mode(OutputMode.POLLING)
    print("[信息] 模式: 问答式")

    # 读取5次，每次间隔1秒
    for i in range(5):
        data = compass.read()
        if data:
            print(f"  [{i+1}] Roll={data.roll:+7.2f}°  "
                  f"Pitch={data.pitch:+7.2f}°  "
                  f"Heading={data.heading:6.2f}°")
        else:
            print(f"  [{i+1}] 读取超时")
        time.sleep(1)

    compass.disconnect()
    print("[完成]\n")


def example_2_continuous_mode():
    """
    示例2: 自动输出模式（5Hz）
    - 设置后传感器自动发送数据
    - 适合实时显示
    """
    print("=" * 55)
    print("示例2: 自动输出模式（5Hz）")
    print("=" * 55)

    port = get_com_port()
    compass = DDM350B(port)

    if not compass.connect():
        print(f"[错误] 无法打开串口 {port}")
        return

    # 设置自动输出模式
    compass.set_mode(OutputMode.AUTO_5HZ)
    print("[信息] 模式: 自动输出 5Hz")

    # 连续读取10次
    count = 0
    for data in compass.read_continuous(interval=0.2):
        print(f"  [{count+1:2d}] Roll={data.roll:+7.2f}°  "
              f"Pitch={data.pitch:+7.2f}°  "
              f"Heading={data.heading:6.2f}°")
        count += 1
        if count >= 100:
            break

    compass.disconnect()
    print("[完成]\n")


def example_3_high_speed():
    """
    示例3: 高速模式（100Hz）
    - 适合需要高频采样的应用
    - 数据采集后计算平均值
    """
    print("=" * 55)
    print("示例3: 高速模式（100Hz）- 数据采集")
    print("=" * 55)

    port = get_com_port()
    compass = DDM350B(port)

    if not compass.connect():
        print(f"[错误] 无法打开串口 {port}")
        return

    compass.set_mode(OutputMode.AUTO_100HZ)
    print("[信息] 模式: 自动输出 100Hz")
    print("[信息] 正在采集数据...")

    # 采集100个样本
    samples = []
    start_time = time.time()

    for data in compass.read_continuous(interval=0.01):
        samples.append(data)
        if len(samples) >= 100:
            break

    elapsed = time.time() - start_time

    compass.disconnect()

    # 计算统计信息
    if samples:
        avg_roll = sum(d.roll for d in samples) / len(samples)
        avg_pitch = sum(d.pitch for d in samples) / len(samples)
        avg_heading = sum(d.heading for d in samples) / len(samples)

        print(f"[统计] 采集样本数: {len(samples)}")
        print(f"[统计] 耗时: {elapsed:.2f}秒")
        print(f"[统计] 实际频率: {len(samples)/elapsed:.1f}Hz")
        print(f"[平均] Roll={avg_roll:+7.2f}°  "
              f"Pitch={avg_pitch:+7.2f}°  "
              f"Heading={avg_heading:6.2f}°")
    print("[完成]\n")


def example_4_stream_to_file():
    """
    示例4: 数据流写入文件
    - 将实时数据保存为CSV格式
    """
    print("=" * 55)
    print("示例4: 数据流写入文件")
    print("=" * 55)

    port = get_com_port()
    output_file = "compass_data.csv"

    compass = DDM350B(port)
    if not compass.connect():
        print(f"[错误] 无法打开串口 {port}")
        return

    compass.set_mode(OutputMode.AUTO_15HZ)

    print(f"[信息] 模式: 自动输出 15Hz")
    print(f"[信息] 保存到: {output_file}")
    print("[提示] 按 Ctrl+C 停止采集\n")

    with open(output_file, 'w') as f:
        f.write("timestamp,roll,pitch,heading\n")

        try:
            for data in compass.read_continuous(interval=0.1):
                timestamp = time.time()
                line = f"{timestamp:.3f},{data.roll:.4f},{data.pitch:.4f},{data.heading:.4f}\n"
                f.write(line)
                f.flush()

                print(f"  Roll={data.roll:+7.2f}°  "
                      f"Pitch={data.pitch:+7.2f}°  "
                      f"Heading={data.heading:6.2f}°")

        except KeyboardInterrupt:
            print("\n[停止] 用户中断")

    compass.disconnect()
    print(f"[完成] 数据已保存到 {output_file}\n")


def example_5_data_class():
    """
    示例5: 使用 CompassData 命名元组
    - 数据以命名元组形式返回
    - 可通过索引或属性名访问
    """
    print("=" * 55)
    print("示例5: CompassData 数据结构")
    print("=" * 55)

    port = get_com_port()
    compass = DDM350B(port)

    if not compass.connect():
        print(f"[错误] 无法打开串口 {port}")
        return

    compass.set_mode(OutputMode.POLLING)

    data = compass.read()
    if data:
        # 方式1: 属性访问
        print(f"属性访问:")
        print(f"  data.roll    = {data.roll}")
        print(f"  data.pitch   = {data.pitch}")
        print(f"  data.heading = {data.heading}")

        # 方式2: 索引访问
        print(f"\n索引访问:")
        print(f"  data[0] = {data[0]}")
        print(f"  data[1] = {data[1]}")
        print(f"  data[2] = {data[2]}")

        # 方式3: 解包
        print(f"\n解包:")
        roll, pitch, heading = data
        print(f"  roll={roll}, pitch={pitch}, heading={heading}")

    compass.disconnect()
    print("[完成]\n")


def run_all_examples():
    """运行所有示例"""
    print("\n" + "#" * 55)
    print("#  DDM350B/DDM360B 三维电子罗盘示例程序")
    print("#" * 55)
    print("\n提示: 如果只需要运行某个示例，请直接调用对应的函数\n")

    #example_1_polling_mode()
    
    example_2_continuous_mode()
    #example_3_high_speed()
    #example_4_stream_to_file()
    example_5_data_class()

if __name__ == "__main__":
    run_all_examples()
