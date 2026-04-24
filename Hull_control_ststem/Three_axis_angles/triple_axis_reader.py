import serial
import time
from typing import Optional, Tuple, List


# ============ 模式定义 ============
class SensorMode:
    """传感器模式常量"""
    # 模式值与命令映射
    MODES = {
        0x00: (b'\x68\x04\x00\x00\x04', "问答模式"),
        0x01: (b'\x68\x04\x00\x01\x05', "主动输出模式1"),
        0x02: (b'\x68\x04\x00\x02\x06', "主动输出模式2"),
        0x03: (b'\x68\x04\x00\x03\x07', "主动输出模式3"),
        0x04: (b'\x68\x04\x00\x04\x08', "100Hz高速模式"),
        0x05: (b'\x68\x04\x00\x05\x09', "50Hz中速模式"),
        0x06: (b'\x68\x04\x00\x06\x0A', "20Hz低速模式"),
    }
    # 常用模式快捷方式
    POLLING = 0x00      # 问答模式
    HIGH_SPEED = 0x04   # 100Hz
    MEDIUM_SPEED = 0x05 # 50Hz
    LOW_SPEED = 0x06    # 20Hz


class SerialBuffer:
    """
    串口数据缓冲区管理，用于避免数据错位
    """
    
    def __init__(self):
        self.buffer = bytearray()
    
    def clear(self):
        """清除所有缓冲数据"""
        self.buffer.clear()
    
    def append(self, data: bytes):
        """追加数据到缓冲区"""
        self.buffer.extend(data)
    
    def find_frame(self, min_length: int = 14) -> bytes | None:
        """查找并返回完整的帧"""
        if len(self.buffer) < min_length:
            return None
        
        try:
            start_idx = self.buffer.index(0x68)
        except ValueError:
            if len(self.buffer) > 1:
                self.buffer = self.buffer[-1:]
            return None
        
        if start_idx > 0:
            self.buffer = self.buffer[start_idx:]
        
        if len(self.buffer) < 2:
            return None
        
        frame_length = self.buffer[1]
        total_len = frame_length + 2
        
        if len(self.buffer) >= total_len:
            frame = bytes(self.buffer[:total_len])
            self.buffer = self.buffer[total_len:]
            return frame
        
        return None
    
    def flush_input(self, ser: serial.Serial):
        """清除串口输入缓冲区"""
        if ser and ser.is_open:
            ser.reset_input_buffer()
        self.clear()


def parse_angle_byte(b1: int, b2: int, b3: int) -> float:
    """
    解析单个轴的角度
    
    三个字节拆成6个十进制数字
    """
    d1 = (b1 >> 4) & 0x0F
    d2 = b1 & 0x0F
    d3 = (b2 >> 4) & 0x0F
    d4 = b2 & 0x0F
    d5 = (b3 >> 4) & 0x0F
    d6 = b3 & 0x0F
    
    sign = -1 if d1 == 1 else 1
    value = (d2 * 100) + (d3 * 10) + d4 + (d5 * 0.1) + (d6 * 0.01)
    
    return sign * value


def parse_data_frame(data: bytes) -> Optional[Tuple[float, float, float]]:
    """解析14字节数据帧"""
    if len(data) < 14 or data[0] != 0x68:
        return None
    
    group1 = data[4:7]
    group2 = data[7:10]
    group3 = data[10:13]
    
    return (
        parse_angle_byte(*group1),
        parse_angle_byte(*group2),
        parse_angle_byte(*group3)
    )


class TripleAxisReader:
    """
    三轴角度传感器读取器
    
    使用示例:
        # 方式1: 函数式调用
        axis_angles = axis(port='/dev/ttyS0', model=1)
        
        # 方式2: 面向对象
        with TripleAxisReader('/dev/ttyS0') as reader:
            reader.set_mode(1)
            angles = reader.read()
    """
    
    def __init__(self, port: str = '/dev/ttyS0', baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self._buffer = SerialBuffer()
        self._current_mode = 0x00
        self._history: List[Tuple[float, float, float]] = []
        self._max_history = 100
    
    def open(self) -> bool:
        """打开串口连接"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self._buffer.flush_input(self.ser)
            return True
        except serial.SerialException as e:
            print(f"打开串口失败: {e}")
            return False
    
    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
    
    def set_mode(self, mode: int) -> bool:
        """设置传感器模式 (0-6)"""
        if mode not in SensorMode.MODES:
            print(f"无效模式: {mode}, 有效范围: 0-6")
            return False
        
        self._current_mode = mode
        cmd = SensorMode.MODES[mode][0]
        
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd)
                self._buffer.flush_input(self.ser)
            except serial.SerialException as e:
                print(f"设置模式失败: {e}")
                return False
        return True
    
    def read(self, timeout: float = 0.1) -> Optional[Tuple[float, float, float]]:
        """读取一组三轴角度"""
        if not self.ser or not self.ser.is_open:
            return None
        
        # 问答模式需要主动请求
        if self._current_mode == SensorMode.POLLING:
            cmd = SensorMode.MODES[SensorMode.POLLING][0]
            self.ser.write(cmd)
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self._buffer.append(data)
                frame = self._buffer.find_frame()
                if frame:
                    angles = parse_data_frame(frame)
                    if angles:
                        self._history.append(angles)
                        if len(self._history) > self._max_history:
                            self._history.pop(0)
                        return angles
            time.sleep(0.001)
        
        return None
    
    def read_continuous(self, count: int = 1, interval: float = 0.01) -> List[Tuple[float, float, float]]:
        """连续读取多组数据"""
        results = []
        for _ in range(count):
            angles = self.read(timeout=interval * 2)
            if angles:
                results.append(angles)
            time.sleep(interval)
        return results
    
    def get_history(self, count: Optional[int] = None) -> List[Tuple[float, float, float]]:
        """获取历史数据"""
        if count is None:
            return self._history.copy()
        return self._history[-count:] if count <= len(self._history) else self._history.copy()
    
    def clear_history(self):
        """清除历史数据"""
        self._history.clear()
    
    def __enter__(self):
        self.open()
        return self
    
    def __exit__(self, *args):
        self.close()


# ============ 全局便捷函数 ============
_global_sensor: Optional[TripleAxisReader] = None


def axis(port: str = '/dev/ttyS0', baudrate: int = 115200, model: int = 4,
         timeout: float = 0.1) -> Optional[Tuple[float, float, float]]:
    """
    读取三轴角度的便捷函数
    
    Args:
        port: 串口名
        baudrate: 波特率
        model: 输出模式 (0-6)
              0: 问答模式
              1-3: 主动输出模式
              4: 100Hz高速模式
              5: 50Hz中速模式
              6: 20Hz低速模式
        timeout: 读取超时（秒）
    
    Returns:
        (轴1角度, 轴2角度, 轴3角度) 或 None
        
    使用示例:
        axis_angles = axis(model=1)
        if axis_angles:
            x, y, z = axis_angles
    """
    global _global_sensor
    
    if _global_sensor is None:
        _global_sensor = TripleAxisReader(port=port, baudrate=baudrate)
        if not _global_sensor.open():
            _global_sensor = None
            return None
    
    if _global_sensor.port != port or _global_sensor.baudrate != baudrate:
        _global_sensor.close()
        _global_sensor = TripleAxisReader(port=port, baudrate=baudrate)
        if not _global_sensor.open():
            _global_sensor = None
            return None
    
    _global_sensor.set_mode(model)
    return _global_sensor.read(timeout=timeout)


def axis_continuous(port: str = '/dev/ttyS0', baudrate: int = 115200, model: int = 4,
                    count: int = 10, interval: float = 0.01) -> List[Tuple[float, float, float]]:
    """
    连续读取多组三轴角度
    
    使用示例:
        data = axis_continuous(model=4, count=100)
    """
    global _global_sensor
    
    if _global_sensor is None:
        _global_sensor = TripleAxisReader(port=port, baudrate=baudrate)
        if not _global_sensor.open():
            _global_sensor = None
            return []
    
    _global_sensor.set_mode(model)
    return _global_sensor.read_continuous(count=count, interval=interval)


def axis_close():
    """关闭全局传感器连接"""
    global _global_sensor
    if _global_sensor:
        _global_sensor.close()
        _global_sensor = None


# ============ 原有函数（保留兼容性） ============
_buffer = SerialBuffer()


def flush_and_clear(ser: serial.Serial):
    """清除串口缓冲区和内部缓冲区"""
    if ser and ser.is_open:
        ser.reset_input_buffer()
    _buffer.clear()
    print("  └ 包缓冲区已清除")


def read_serial_data(port: str = '/dev/ttyS0', baudrate: int = 115200,
                     request_cmd: bytes = b'\x68\x04\x00\x04\x08',
                     request_interval: float = 0.05):
    """持续发送请求并解析数据"""
    
    print("=" * 60)
    print("三轴传感器数据读取程序")
    print(f"串口: {port} | 波特率: {baudrate}")
    print("=" * 60)
    
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1.0,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        flush_and_clear(ser)
        print(f"✓ 串口 {port} 打开成功")
        print("-" * 60)
        print("开始读取数据... (按 Ctrl+C 退出)\n")
        
        while True:
            ser.write(request_cmd)
            time.sleep(request_interval)
            
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                _buffer.append(data)
            
            frames = []
            while len(_buffer.buffer) >= 14:
                try:
                    start_idx = _buffer.buffer.index(0x68)
                except ValueError:
                    if len(_buffer.buffer) > 1:
                        _buffer.buffer = _buffer.buffer[-1:]
                    break
                
                if start_idx > 0:
                    _buffer.buffer = _buffer.buffer[start_idx:]
                
                if len(_buffer.buffer) < 2:
                    break
                
                frame_length = _buffer.buffer[1]
                total_len = frame_length + 2
                
                if len(_buffer.buffer) >= total_len:
                    frame = bytes(_buffer.buffer[:total_len])
                    _buffer.buffer = _buffer.buffer[total_len:]
                    frames.append(frame)
                else:
                    break
            
            for frame in frames:
                hex_str = ' '.join(f'{b:02X}' for b in frame)
                if len(frame) >= 14:
                    print(f"原始: {hex_str}")
                    print(f"  组1: {frame[4]:02X} {frame[5]:02X} {frame[6]:02X}")
                    print(f"  组2: {frame[7]:02X} {frame[8]:02X} {frame[9]:02X}")
                    print(f"  组3: {frame[10]:02X} {frame[11]:02X} {frame[12]:02X}")
                    
                    angles = parse_data_frame(frame)
                    if angles:
                        print(f"  → 轴1: {angles[0]:>+8.2f}°")
                        print(f"  → 轴2: {angles[1]:>+8.2f}°")
                        print(f"  → 轴3: {angles[2]:>+8.2f}°")
                    print("-" * 40)
            
    except serial.SerialException as e:
        print(f"\n✗ 串口错误: {e}")
    except KeyboardInterrupt:
        print("\n\n✓ 程序已退出")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


def test_parser():
    """测试解析器"""
    print("\n" + "=" * 60)
    print("解析器测试")
    print("=" * 60)
    
    test_data = bytes([0x68, 0x0D, 0x00, 0x84, 0x10, 0x02, 0x36, 0x00, 0x04, 0x35, 0x01, 0x28, 0x55, 0x90])
    print(f"\n数据: {' '.join(f'{b:02X}' for b in test_data)}")
    
    for idx, start in enumerate([4, 7, 10], 1):
        b1, b2, b3 = test_data[start], test_data[start+1], test_data[start+2]
        d1 = (b1 >> 4) & 0x0F
        d2 = b1 & 0x0F
        d3 = (b2 >> 4) & 0x0F
        d4 = b2 & 0x0F
        d5 = (b3 >> 4) & 0x0F
        d6 = b3 & 0x0F
        
        sign_str = "负" if d1 == 1 else "正"
        value = (d2 * 100) + (d3 * 10) + d4 + (d5 * 0.1) + (d6 * 0.01)
        print(f"  轴{idx}: 符号:{sign_str}, 数值:{value:.2f}°")
    
    angles = parse_data_frame(test_data)
    if angles:
        print(f"结果: 轴1={angles[0]:>+7.2f}°, 轴2={angles[1]:>+7.2f}°, 轴3={angles[2]:>7.2f}°")


if __name__ == "__main__":
    test_parser()
    print("\n" + "=" * 60)
    print("开始实时读取...")
    print("=" * 60 + "\n")
    
    read_serial_data(
        port='/dev/ttyS0',
        baudrate=115200,
        request_cmd=b'\x68\x04\x00\x04\x08',
        request_interval=0.05
    )
