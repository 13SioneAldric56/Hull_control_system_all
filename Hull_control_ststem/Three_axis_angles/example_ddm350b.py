"""
DDM350B/DDM360B 三维电子罗盘使用示例
"""

from ddm350b import DDM350B, OutputMode, read_compass, close


def example_basic():
    """基础用法：问答模式读取"""
    print("=" * 50)
    print("示例1: 问答模式读取")
    print("=" * 50)
    
    with DDM350B('/dev/ttyS0') as compass:
        if not compass.is_connected():
            print("无法连接到罗盘")
            return
        
        # 设置为问答模式
        compass.set_mode(OutputMode.POLLING)
        
        # 读取10次数据
        for i in range(10):
            data = compass.read()
            if data:
                print(f"[{i+1:2d}] Roll: {data.roll:>+7.2f}°  "
                      f"Pitch: {data.pitch:>+7.2f}°  "
                      f"Heading: {data.heading:>6.2f}°")
            else:
                print(f"[{i+1:2d}] 读取失败")
            compass._buffer.clear()  # 清除缓冲区避免数据堆积


def example_continuous():
    """示例2: 自动输出模式"""
    print("\n" + "=" * 50)
    print("示例2: 自动输出模式 (10Hz)")
    print("=" * 50)
    
    compass = DDM350B('/dev/ttyS0', baudrate=115200)
    if not compass.connect():
        print("无法连接到罗盘")
        return
    
    # 设置为自动输出10Hz模式
    compass.set_mode(OutputMode.AUTO_50HZ)
    
    # 连续读取5次
    count = 0
    for data in compass.read_continuous(interval=0.1):
        print(f"Roll: {data.roll:>+7.2f}°  "
              f"Pitch: {data.pitch:>+7.2f}°  "
              f"Heading: {data.heading:>6.2f}°")
        count += 1
        if count >= 99999:
            break
    
    compass.disconnect()


def example_quick():
    """示例3: 快捷函数"""
    print("\n" + "=" * 50)
    print("示例3: 快捷函数")
    print("=" * 50)
    
    # 单次读取
    data = read_compass('/dev/ttyS0', mode=OutputMode.POLLING)
    if data:
        print(f"横滚角 (Roll): {data.roll:+.2f}°")
        print(f"俯仰角 (Pitch): {data.pitch:+.2f}°")
        print(f"航向角 (Heading): {data.heading:.2f}°")
    else:
        print("读取失败")
    
    close()  # 关闭连接


def example_calibration():
    """示例4: 校准流程"""
    print("\n" + "=" * 50)
    print("示例4: 校准流程")
    print("=" * 50)
    print("提示: 校准时需要将罗盘水平放置并旋转360°")
    
    compass = DDM350B('/dev/ttyS0')
    if not compass.connect():
        print("无法连接到罗盘")
        return
    
    input("确保罗盘水平放置后按 Enter 开始校准...")
    
    # 开始校准
    if compass.start_calibration():
        print("✓ 校准已开始，现在水平旋转罗盘360°...")
        input("旋转完成后按 Enter 保存校准...")
        
        # 保存校准
        if compass.save_calibration():
            print("✓ 校准已保存")
        else:
            print("✗ 校准保存失败")
    else:
        print("✗ 校准开始失败")
    
    compass.disconnect()


if __name__ == "__main__":
    print("DDM350B/DDM360B 三维电子罗盘示例程序")
    print("-" * 50)
    print("请根据实际串口号修改 /dev/ttyS0")
    print()
    
    # 运行所有示例
    #example_basic()
    example_continuous()  # 取消注释以运行连续读取示例
    # example_quick()       # 取消注释以运行快捷函数示例
    # example_calibration() # 取消注释以运行校准示例
