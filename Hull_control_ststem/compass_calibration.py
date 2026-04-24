#!/usr/bin/env python3
"""
电子罗盘校准程序
用于校准 DDM350B 三轴电子罗盘

协议说明:
- 开始校准: 68 04 00 08 0C 50
  - 68: 帧头
  - 04: 数据长度
  - 00 08: 开始校准命令
  - 0C: 校验和 (04 XOR 00 XOR 08)
  - 50: 校准持续时间 (50秒)

- 结束校准: 68 04 00 0A 0E
  - 68: 帧头
  - 04: 数据长度
  - 00 0A: 结束校准命令
  - 0E: 校验和 (04 XOR 00 XOR 0A)

校准步骤:
1. 发送开始校准命令
2. 在50秒内，手动将罗盘在各个方向旋转（8字形或360度旋转）
3. 等待50秒后自动发送结束校准命令

使用说明:
    python compass_calibration.py
    python compass_calibration.py /dev/ttyS0 -b 115200
"""

import serial
import time
import sys
import argparse
import os


# ===================== 校准命令定义 =====================
CALIBRATION_START_CMD = bytes([0x68, 0x04, 0x00, 0x08, 0x0C])
CALIBRATION_END_CMD = bytes([0x68, 0x04, 0x00, 0x0A, 0x0E])


# ===================== 默认配置 =====================
DEFAULT_PORT = "/dev/ttyS0"
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 1.0
CALIBRATION_DURATION = 50  # 秒


def clear_input_buffer(ser):
    """清空串口缓冲区"""
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except:
        pass


def send_command(ser, data: bytes) -> bool:
    """
    发送串口命令
    
    Args:
        ser: 串口对象
        data: 要发送的数据
    
    Returns:
        是否发送成功
    """
    try:
        clear_input_buffer(ser)
        bytes_written = ser.write(data)
        ser.flush()
        print(f"[发送] {bytes_written} 字节: {' '.join(f'{b:02X}' for b in data)}")
        return bytes_written == len(data)
    except Exception as e:
        print(f"[错误] 发送失败: {e}")
        return False


def read_response(ser, timeout: float = 1.0) -> bytes:
    """
    读取串口响应
    
    Args:
        ser: 串口对象
        timeout: 超时时间
    
    Returns:
        接收到的数据
    """
    try:
        time.sleep(0.1)
        # 设置临时超时
        original_timeout = ser.timeout
        ser.timeout = timeout
        
        # 读取所有可用数据
        data = ser.read(1024)
        
        # 恢复超时设置
        ser.timeout = original_timeout
        
        return data
    except Exception as e:
        print(f"[错误] 读取失败: {e}")
        return b''


def print_progress_bar(iteration: int, total: int, prefix: str = '', 
                        suffix: str = '', length: int = 50, fill: str = '█'):
    """
    打印进度条
    
    Args:
        iteration: 当前迭代
        total: 总迭代
        prefix: 前缀字符串
        suffix: 后缀字符串
        length: 进度条长度
        fill: 填充字符
    """
    percent = 100 * (iteration / float(total))
    filled_length = int(length * iteration // total)
    bar = fill * filled_length + '-' * (length - filled_length)
    # 使用 \r 回到行首，不换行
    print(f'\r{prefix} |{bar}| {percent:.1f}% {suffix}', end='', flush=True)


def wait_for_countdown(seconds: int):
    """
    倒计时等待
    
    Args:
        seconds: 等待秒数
    """
    print(f"\n{'='*60}")
    print(f"  校准进行中，请旋转罗盘（建议做8字形或360度旋转）")
    print(f"{'='*60}\n")
    
    for i in range(seconds, 0, -1):
        print(f"\r[{time.strftime('%H:%M:%S')}] 剩余 {i:2d} 秒...", end='', flush=True)
        time.sleep(1)
    
    print("\n")


def compass_calibration(port: str = DEFAULT_PORT, 
                       baudrate: int = DEFAULT_BAUDRATE,
                       timeout: float = DEFAULT_TIMEOUT,
                       duration: int = CALIBRATION_DURATION,
                       dry_run: bool = False) -> bool:
    """
    执行电子罗盘校准
    
    Args:
        port: 串口路径
        baudrate: 波特率
        timeout: 串口超时时间
        duration: 校准持续时间（秒）
        dry_run: 是否为模拟模式（不实际发送数据）
    
    Returns:
        校准是否成功
    """
    print(f"\n{'#'*60}")
    print(f"#  电子罗盘 (DDM350B) 校准程序")
    print(f"{'#'*60}")
    print(f"\n  串口: {port}")
    print(f"  波特率: {baudrate}")
    print(f"  校准时长: {duration} 秒")
    print(f"  模式: {'模拟' if dry_run else '实际'}")
    print(f"\n{'#'*60}\n")

    if dry_run:
        print("[模拟模式] 将在 5 秒后开始模拟校准...\n")
        time.sleep(5)
        print("[模拟] 发送开始校准命令: 68 04 00 08 0C 50")
        for i in range(duration, 0, -1):
            print(f"\r[模拟] 剩余 {i:2d} 秒...", end='', flush=True)
            time.sleep(0.05)
        print("\n[模拟] 发送结束校准命令: 68 04 00 0A 0E")
        print("\n✓ 模拟校准完成！")
        return True

    # 实际校准模式
    ser = None
    try:
        # 打开串口
        print(f"[连接] 正在打开串口 {port}...")
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        print(f"[连接] 串口已打开\n")

        # 发送开始校准命令
        print("=" * 60)
        print("  步骤 1: 发送开始校准命令")
        print("=" * 60)
        if not send_command(ser, CALIBRATION_START_CMD):
            return False

        # 读取响应
        response = read_response(ser)
        if response:
            print(f"[接收] {len(response)} 字节: {' '.join(f'{b:02X}' for b in response)}")

        # 等待校准时间
        wait_for_countdown(duration)

        # 发送结束校准命令
        print("=" * 60)
        print("  步骤 2: 发送结束校准命令")
        print("=" * 60)
        if not send_command(ser, CALIBRATION_END_CMD):
            return False

        # 读取响应
        response = read_response(ser)
        if response:
            print(f"[接收] {len(response)} 字节: {' '.join(f'{b:02X}' for b in response)}")

        print("\n" + "=" * 60)
        print("  ✓ 校准完成！")
        print("=" * 60)

        return True

    except serial.SerialException as e:
        print(f"\n[错误] 串口错误: {e}")
        print(f"\n请检查:")
        print(f"  1. 串口 {port} 是否存在")
        print(f"  2. 串口是否被其他程序占用")
        print(f"  3. 波特率 {baudrate} 是否正确")
        print(f"  4. 接线是否正确 (TX/RX 交叉连接)")
        return False

    except PermissionError as e:
        print(f"\n[错误] 权限不足: {e}")
        print(f"\n解决方案:")
        print(f"  sudo usermod -a -G dialout $USER")
        print(f"  或者使用 sudo 运行此程序")
        return False

    except KeyboardInterrupt:
        print("\n\n[中断] 用户取消校准")
        print("[提示] 如果校准正在进行中，请手动发送结束命令以确保罗盘状态正确")
        return False

    except Exception as e:
        print(f"\n[错误] 未知错误: {e}")
        return False

    finally:
        if ser and ser.is_open:
            ser.close()
            print(f"[关闭] 串口已关闭")


def interactive_calibration():
    """
    交互式校准模式
    """
    print(f"\n{'#'*60}")
    print(f"#  电子罗盘交互式校准")
    print(f"{'#'*60}\n")
    
    print("可用命令:")
    print("  1 - 开始校准 (发送 68 04 00 08 0C 50)")
    print("  2 - 结束校准 (发送 68 04 00 0A 0E)")
    print("  3 - 读取罗盘数据")
    print("  4 - 发送自定义命令")
    print("  5 - 循环校准 (自动开始+50秒+结束)")
    print("  q - 退出程序")
    print()
    
    port = input(f"请输入串口路径 [{DEFAULT_PORT}]: ").strip() or DEFAULT_PORT
    baudrate_input = input(f"请输入波特率 [{DEFAULT_BAUDRATE}]: ").strip()
    baudrate = int(baudrate_input) if baudrate_input else DEFAULT_BAUDRATE
    
    ser = None
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )
        print(f"\n[连接] 串口 {port} 已打开 (波特率: {baudrate})\n")
        
        while True:
            cmd = input("请输入命令 (1/2/3/4/5/q): ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == '1':
                print("\n发送开始校准命令...")
                send_command(ser, CALIBRATION_START_CMD)
            elif cmd == '2':
                print("\n发送结束校准命令...")
                send_command(ser, CALIBRATION_END_CMD)
            elif cmd == '3':
                print("\n读取罗盘数据...")
                response = read_response(ser)
                if response:
                    print(f"接收: {' '.join(f'{b:02X}' for b in response)}")
                else:
                    print("无数据响应")
            elif cmd == '4':
                hex_input = input("输入十六进制数据 (空格分隔, 如: 68 04 00 08): ").strip()
                try:
                    data = bytes([int(b, 16) for b in hex_input.split()])
                    send_command(ser, data)
                except ValueError:
                    print("输入格式错误！")
            elif cmd == '5':
                print("\n开始循环校准...")
                send_command(ser, CALIBRATION_START_CMD)
                wait_for_countdown(50)
                send_command(ser, CALIBRATION_END_CMD)
            else:
                print("无效命令！")
    
    except serial.SerialException as e:
        print(f"\n[错误] 串口错误: {e}")
    except KeyboardInterrupt:
        print("\n\n退出程序")
    finally:
        if ser and ser.is_open:
            ser.close()


def main():
    parser = argparse.ArgumentParser(
        description='电子罗盘校准程序',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  基本校准:
    python compass_calibration.py
  
  指定串口和波特率:
    python compass_calibration.py /dev/ttyS0 -b 9600
  
  模拟模式 (不实际发送数据):
    python compass_calibration.py --dry-run
  
  交互式模式:
    python compass_calibration.py -i

校准步骤:
  1. 运行此程序或使用交互模式
  2. 程序会自动发送开始校准命令
  3. 在 50 秒内，将罗盘做 8 字形或 360 度旋转
  4. 程序会在 50 秒后自动发送结束校准命令
  5. 校准完成
        """
    )
    
    parser.add_argument('port', nargs='?', default=DEFAULT_PORT,
                       help=f'串口路径 (默认: {DEFAULT_PORT})')
    parser.add_argument('-b', '--baudrate', type=int, default=DEFAULT_BAUDRATE,
                       help=f'波特率 (默认: {DEFAULT_BAUDRATE})')
    parser.add_argument('-t', '--timeout', type=float, default=DEFAULT_TIMEOUT,
                       help=f'串口超时时间 (默认: {DEFAULT_TIMEOUT})')
    parser.add_argument('-d', '--duration', type=int, default=CALIBRATION_DURATION,
                       help=f'校准持续时间(秒) (默认: {CALIBRATION_DURATION})')
    parser.add_argument('--dry-run', action='store_true',
                       help='模拟模式，不实际发送数据')
    parser.add_argument('-i', '--interactive', action='store_true',
                       help='交互式模式')
    parser.add_argument('-l', '--list', action='store_true',
                       help='列出可用串口')
    
    args = parser.parse_args()
    
    # 列出串口
    if args.list:
        print("可用串口:")
        ports = serial.tools.list_ports.comports()
        if ports:
            for p in ports:
                print(f"  {p.device} - {p.description}")
        else:
            print("  未找到可用串口")
        print()
        return
    
    # 交互式模式
    if args.interactive:
        interactive_calibration()
        return
    
    # 执行校准
    success = compass_calibration(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        duration=args.duration,
        dry_run=args.dry_run
    )
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
