"""
DDM350B/DDM360B 三维电子罗盘 Python 接口

基于瑞芬科技 DDM350B/DDM360B 高精度动态三维电子罗盘通讯协议

使用示例:
    # 方式1: 快速读取
    from ddm350b import DDM350B
    compass = DDM350B('COM3')
    compass.connect()
    roll, pitch, heading = compass.read()
    
    # 方式2: 自动输出模式
    compass = DDM350B('COM3', mode='continuous_10hz')
    compass.connect()
    for roll, pitch, heading in compass:
        print(f"R:{roll:.1f} P:{pitch:.1f} H:{heading:.1f}")
    
    # 方式3: 设置磁偏角
    compass.set_magnetic_declination(8.5)
"""

import serial
import time
from typing import Optional, Tuple, NamedTuple
from enum import IntEnum


class OutputMode(IntEnum):
    """输出模式枚举"""
    POLLING = 0x00      # 问答式（默认）
    AUTO_5HZ = 0x01     # 自动输出 5Hz
    AUTO_15HZ = 0x02    # 自动输出 15Hz
    AUTO_25HZ = 0x03    # 自动输出 25Hz
    AUTO_35HZ = 0x04    # 自动输出 35Hz
    AUTO_50HZ = 0x05    # 自动输出 50Hz
    AUTO_100HZ = 0x06   # 自动输出 100Hz


class CompassData(NamedTuple):
    """罗盘数据结构"""
    roll: float      # 横滚角 (°)
    pitch: float     # 俯仰角 (°)
    heading: float   # 航向角 (°)


class DDM350B:
    """
    DDM350B/DDM360B 三维电子罗盘接口类
    
    Attributes:
        port: 串口名，如 'COM3' 或 '/dev/ttyUSB0'
        baudrate: 波特率，默认 9600（与PDF文档一致）
        timeout: 串口超时时间（秒）
    """
    
    # 通讯参数默认值（与PDF文档一致）
    DEFAULT_BAUDRATE = 115200
    DEFAULT_TIMEOUT = 1.0
    
    # 帧头和命令字
    FRAME_HEADER = 0x68
    CMD_READ_ANGLES = 0x04
    CMD_SET_MODE = 0x0C
    CMD_SET_BAUDRATE = 0x0B
    CMD_SET_MAG_DECLINATION = 0x06
    CMD_READ_MAG_DECLINATION = 0x07
    CMD_START_CALIBRATION = 0x08
    CMD_SAVE_CALIBRATION = 0x0A
    CMD_SET_ADDRESS = 0x0F
    
    def __init__(self, port: str, baudrate: int = None, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate if baudrate else self.DEFAULT_BAUDRATE
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self._buffer = bytearray()
        self._mode = OutputMode.POLLING
    
    def connect(self) -> bool:
        """
        连接到罗盘传感器
        
        Returns:
            True 连接成功, False 连接失败
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self._clear_buffer()
            return True
        except serial.SerialException as e:
            print(f"连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None
    
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self.serial is not None and self.serial.is_open
    
    def _clear_buffer(self):
        """清除串口缓冲区"""
        if self.serial and self.serial.is_open:
            self.serial.reset_input_buffer()
        self._buffer.clear()
    
    def _calculate_checksum(self, data: bytes) -> int:
        """
        计算校验和
        校验和 = (长度 + 地址码 + 命令字 + 数据域) & 0xFF
        """
        return sum(data) & 0xFF
    
    def _send_command(self, cmd: bytes) -> bool:
        """发送命令并检查响应"""
        if not self.is_connected():
            return False
        try:
            self.serial.write(cmd)
            return True
        except serial.SerialException as e:
            print(f"发送命令失败: {e}")
            return False
    
    def _parse_angle_bytes(self, b1: int, b2: int, b3: int) -> float:
        """
        解析角度数据（三个字节 -> 角度值）
        
        数据格式:
            d1 = (b1 >> 4) & 0x0F → 符号位 (0正, 1负)
            d2 = b1 & 0x0F        → 百位/符号扩展
            d3 = (b2 >> 4) & 0x0F → 十位
            d4 = b2 & 0x0F        → 个位
            d5 = (b3 >> 4) & 0x0F → 十分位
            d6 = b3 & 0x0F        → 百分位
        
        示例: 0x10 0x50 0x23 → +50.23°
        """
        d1 = (b1 >> 4) & 0x0F
        d2 = b1 & 0x0F
        d3 = (b2 >> 4) & 0x0F
        d4 = b2 & 0x0F
        d5 = (b3 >> 4) & 0x0F
        d6 = b3 & 0x0F
        
        sign = -1 if d1 == 1 else 1
        value = d2 * 100 + d3 * 10 + d4 + d5 * 0.1 + d6 * 0.01
        return sign * value
    
    def _find_frame(self) -> Optional[bytes]:
        """从缓冲区中查找完整的数据帧"""
        if len(self._buffer) < 2:
            return None
        
        try:
            start_idx = self._buffer.index(self.FRAME_HEADER)
            if start_idx > 0:
                self._buffer = self._buffer[start_idx:]
        except ValueError:
            if len(self._buffer) > 1:
                self._buffer = self._buffer[-1:]
            return None
        
        if len(self._buffer) < 2:
            return None
        
        frame_length = self._buffer[1]
        total_len = frame_length + 2
        
        if len(self._buffer) >= total_len:
            frame = bytes(self._buffer[:total_len])
            self._buffer = self._buffer[total_len:]
            return frame
        
        return None
    
    def _read_response(self, timeout: float = None) -> Optional[bytes]:
        """读取响应数据"""
        if not self.is_connected():
            return None
        
        timeout = timeout if timeout is not None else self.timeout
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.serial.in_waiting > 0:
                data = self.serial.read(self.serial.in_waiting)
                self._buffer.extend(data)
                
                frame = self._find_frame()
                if frame:
                    return frame
            time.sleep(0.001)
        
        return None
    
    # ========== 核心功能方法 ==========
    
    def set_mode(self, mode: OutputMode) -> bool:
        """
        设置输出模式
        
        Args:
            mode: OutputMode 枚举值
            
        Returns:
            True 成功, False 失败
        """
        length = 0x05
        address = 0x00
        cmd = self.CMD_SET_MODE
        data = mode
        
        checksum = (length + address + cmd + data) & 0xFF
        packet = bytes([self.FRAME_HEADER, length, address, cmd, data, checksum])
        
        self._clear_buffer()
        if self._send_command(packet):
            response = self._read_response(timeout=0.5)
            if response and response[3] == 0x8C:
                self._mode = mode
                return True
        return False
    
    def read(self) -> Optional[CompassData]:
        """
        读取一次角度数据
        
        问答模式下自动发送读取命令并等待响应
        自动输出模式下直接读取缓冲区中的数据
        
        Returns:
            CompassData(roll, pitch, heading) 或 None
        """
        if not self.is_connected():
            return None
        
        self._clear_buffer()
        
        if self._mode == OutputMode.POLLING:
            cmd = bytes([self.FRAME_HEADER, 0x04, 0x00, self.CMD_READ_ANGLES, 0x08])
            self._send_command(cmd)
        
        response = self._read_response()
        if not response or response[3] != 0x84:
            return None
        
        roll = self._parse_angle_bytes(response[4], response[5], response[6])
        pitch = self._parse_angle_bytes(response[7], response[8], response[9])
        heading = self._parse_angle_bytes(response[10], response[11], response[12])
        
        return CompassData(roll=roll, pitch=pitch, heading=heading)
    
    def read_continuous(self, interval: float = 0.1):
        """
        连续读取生成器
        
        Args:
            interval: 每次读取间隔（秒）
            
        Yields:
            CompassData(roll, pitch, heading)
            
        Example:
            for data in compass.read_continuous(0.1):
                print(data.heading)
        """
        if not self.is_connected():
            return
        
        while self.serial and self.serial.is_open:
            data = self.read()
            if data:
                yield data
            time.sleep(interval)
    
    def set_magnetic_declination(self, degrees: float) -> bool:
        """
        设置磁偏角
        
        Args:
            degrees: 磁偏角度数，正数为东偏，负数为西偏
            
        Returns:
            True 成功, False 失败
        """
        sign = 0x80 if degrees < 0 else 0x00
        abs_deg = abs(degrees)
        int_part = int(abs_deg)
        dec_part = int((abs_deg - int_part) * 10)
        
        data1 = sign | (int_part // 10 << 4) | (int_part % 10)
        data2 = (dec_part << 4)
        
        length = 0x06
        address = 0x00
        cmd = self.CMD_SET_MAG_DECLINATION
        checksum = (length + address + cmd + data1 + data2) & 0xFF
        
        packet = bytes([self.FRAME_HEADER, length, address, cmd, data1, data2, checksum])
        
        self._clear_buffer()
        if self._send_command(packet):
            response = self._read_response(timeout=0.5)
            if response and response[4] == 0x00:
                return True
        return False
    
    def start_calibration(self) -> bool:
        """开始校准"""
        cmd = bytes([self.FRAME_HEADER, 0x04, 0x00, self.CMD_START_CALIBRATION, 0x0C])
        self._clear_buffer()
        if self._send_command(cmd):
            response = self._read_response(timeout=0.5)
            return response and response[4] == 0x00
        return False
    
    def save_calibration(self) -> bool:
        """保存校准结果"""
        cmd = bytes([self.FRAME_HEADER, 0x04, 0x00, self.CMD_SAVE_CALIBRATION, 0x0E])
        self._clear_buffer()
        if self._send_command(cmd):
            response = self._read_response(timeout=0.5)
            return response and response[4] == 0x00
        return False
    
    def set_baudrate(self, baudrate_code: int) -> bool:
        """
        设置波特率
        
        Args:
            baudrate_code: 波特率代码
                0x01: 4800
                0x02: 9600 (默认)
                0x03: 19200
                0x04: 38400
                0x05: 115200
                
        Returns:
            True 成功, False 失败
        """
        length = 0x05
        address = 0x00
        cmd = self.CMD_SET_BAUDRATE
        checksum = (length + address + cmd + baudrate_code) & 0xFF
        
        packet = bytes([self.FRAME_HEADER, length, address, cmd, baudrate_code, checksum])
        
        self._clear_buffer()
        if self._send_command(packet):
            response = self._read_response(timeout=0.5)
            return response and response[4] == 0x00
        return False
    
    # ========== 上下文管理器支持 ==========
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, *args):
        self.disconnect()


# ========== 快捷函数 ==========

_global_compass: Optional[DDM350B] = None


def read_compass(port: str = 'COM3', mode: OutputMode = OutputMode.POLLING) -> Optional[CompassData]:
    """
    快捷读取函数
    
    Args:
        port: 串口名
        mode: 输出模式
        
    Returns:
        CompassData(roll, pitch, heading) 或 None
        
    Example:
        data = read_compass('COM3')
        if data:
            print(f"航向: {data.heading:.1f}°")
    """
    global _global_compass
    
    if _global_compass is None:
        _global_compass = DDM350B(port)
        if not _global_compass.connect():
            _global_compass = None
            return None
    
    if _global_compass.port != port:
        _global_compass.disconnect()
        _global_compass = DDM350B(port)
        if not _global_compass.connect():
            _global_compass = None
            return None
    
    if _global_compass._mode != mode:
        _global_compass.set_mode(mode)
    
    return _global_compass.read()


def close():
    """关闭全局连接"""
    global _global_compass
    if _global_compass:
        _global_compass.disconnect()
        _global_compass = None
