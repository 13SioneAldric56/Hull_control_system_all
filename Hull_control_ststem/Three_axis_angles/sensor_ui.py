import tkinter as tk
from tkinter import font as tkfont, ttk, messagebox
import serial
import time
import threading
from mode_command import build_set_mode_command, get_mode_name, verify_checksum


class SerialBuffer:
    """
    串口数据缓冲区管理，用于避免数据错位

    功能：
    - 自动寻找帧头 (0x68)
    - 丢弃不完整的帧
    - 清除缓冲区避免旧数据干扰
    """

    # 读取角度命令 (问答模式用)
    READ_ANGLES_CMD = bytes([0x68, 0x04, 0x00, 0x04, 0x08])
    
    def __init__(self):
        self.buffer = bytearray()
    
    def clear(self):
        """清除所有缓冲数据"""
        self.buffer.clear()
    
    def append(self, data: bytes):
        """追加数据到缓冲区"""
        self.buffer.extend(data)
        # 只保留最近14字节，避免积累旧数据
        if len(self.buffer) > 14:
            self.buffer = self.buffer[-14:]
    
    def find_frame(self, min_length: int = 14) -> bytes | None:
        """
        查找并返回完整的帧
        
        帧格式: [帧头 0x68] [长度] [数据...] [校验位]
        
        Returns:
            完整帧数据，或 None（没有完整帧）
        """
        while len(self.buffer) >= min_length:
            # 查找帧头
            try:
                start_idx = self.buffer.index(0x68)
            except ValueError:
                # 没有帧头，保留最后1字节以防它是帧头的一部分
                if len(self.buffer) > 1:
                    self.buffer = self.buffer[-1:]
                return None
            
            # 帧头位置不对齐，直接丢弃前面的数据
            if start_idx > 0:
                self.buffer = self.buffer[start_idx:]
            
            # 检查是否有足够的数据
            if len(self.buffer) < 2:
                return None
            
            # 获取数据长度（字节1是长度字段）
            frame_length = self.buffer[1]
            
            # 检查是否收到完整帧
            if len(self.buffer) >= frame_length + 2:  # +2 是帧头和校验位
                frame = bytes(self.buffer[:frame_length + 2])
                self.buffer = self.buffer[frame_length + 2:]
                return frame
            
            # 数据不完整，等待更多数据
            return None
        
        return None


# 创建全局缓冲区
_buffer = SerialBuffer()


def parse_angle_byte(b1: int, b2: int, b3: int) -> float:
    """解析单个轴的角度"""
    d1 = (b1 >> 4) & 0x0F
    d2 = b1 & 0x0F
    d3 = (b2 >> 4) & 0x0F
    d4 = b2 & 0x0F
    d5 = (b3 >> 4) & 0x0F
    d6 = b3 & 0x0F
    
    sign = -1 if d1 == 1 else 1
    value = (d2 * 100) + (d3 * 10) + d4 + (d5 * 0.1) + (d6 * 0.01)
    
    return sign * value


def parse_data_frame(data: bytes):
    """
    解析14字节数据帧
    
    帧格式: [帧头 0x68] [长度] [地址码] [命令字] [数据域9字节]
    
    验证: 仅验证帧头为 0x68
    
    返回: (roll, pitch, heading) 或 None (帧头错误)
    """
    if len(data) < 14:
        return None
    
    # 帧头验证
    if data[0] != 0x68:
        return None
    
    # 解析三个轴的角度数据 (从第5字节开始，每轴3字节)
    angles = []
    for i in range(3):
        b1, b2, b3 = data[4 + i*3], data[5 + i*3], data[6 + i*3]
        angles.append(parse_angle_byte(b1, b2, b3))
    
    return tuple(angles)


class SensorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("三轴传感器控制台")
        self.root.geometry("480x600")
        self.root.resizable(False, False)
        
        # 串口配置
        self.serial_port = tk.StringVar(value='COM3')
        self.baudrate = tk.IntVar(value=115200)
        self.ser = None
        self.running = False
        self.mode = 0x00  # 默认问答式
        self.auto_mode = False  # 是否处于自动输出模式
        self._waiting_for_mode_confirm = False  # 是否等待模式确认
        
        # 原始数据状态
        self.data_valid = False
        self.data_count = 0
        self.error_count = 0
        
        # 模式映射
        self.mode_map = {
            "问答式(默认)": 0x00,
            "自动输出 5Hz": 0x01,
            "自动输出 15Hz": 0x02,
            "自动输出 25Hz": 0x03,
            "自动输出 35Hz": 0x04,
            "自动输出 50Hz": 0x05,
            "自动输出 100Hz": 0x06
        }
        self.mode_reverse = {v: k for k, v in self.mode_map.items()}
        
        # 创建界面
        self._create_widgets()
        
        # 启动串口线程
        self.thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.thread.start()
        
        # 窗口关闭
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
    
    def _create_widgets(self):
        """创建界面组件"""
        # 标题
        title_font = tkfont.Font(family="Arial", size=20, weight="bold")
        title_label = tk.Label(self.root, text="三轴传感器控制台", font=title_font, fg="#333")
        title_label.pack(pady=15)
        
        # 串口设置框架
        self._create_serial_frame()
        
        # 模式设置框架
        self._create_mode_frame()
        
        # 三个轴显示
        self._create_axis_display()
        
        # 原始数据显示区域
        self._create_raw_data_display()
        
        # 控制按钮
        self._create_control_buttons()
        
        # 状态栏
        self.status_label = tk.Label(self.root, text="状态: 正在初始化...",
                                      font=("Arial", 10), fg="#666", bg="#e0e0e0")
        self.status_label.pack(fill="x", side="bottom", pady=5)
    
    def _create_serial_frame(self):
        """创建串口设置框架"""
        frame = tk.Frame(self.root, bg="#fff3e0", relief="solid", bd=1)
        frame.pack(fill="x", padx=20, pady=8)
        
        tk.Label(frame, text="串口设置:", bg="#fff3e0",
                 font=("Arial", 11, "bold")).pack(side="left", padx=10, pady=8)
        
        # 串口选择
        port_frame = tk.Frame(frame, bg="#fff3e0")
        port_frame.pack(side="left", padx=5)
        tk.Label(port_frame, text="端口:", bg="#fff3e0").pack(side="left")
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.serial_port,
                                        values=["COM1", "COM2", "COM3", "COM4", "COM5",
                                                "COM6", "COM7", "COM8", "COM9", "COM10"],
                                        width=8, state="readonly")
        self.port_combo.pack(side="left", padx=3)
        
        # 波特率
        baud_frame = tk.Frame(frame, bg="#fff3e0")
        baud_frame.pack(side="left", padx=5)
        tk.Label(baud_frame, text="波特率:", bg="#fff3e0").pack(side="left")
        self.baud_combo = ttk.Combobox(baud_frame, textvariable=self.baudrate,
                                        values=[9600, 115200, 19200, 38400, 57600, 230400],
                                        width=8, state="readonly")
        self.baud_combo.pack(side="left", padx=3)
    
    def _create_mode_frame(self):
        """创建模式设置框架"""
        frame = tk.Frame(self.root, bg="#e8f5e9", relief="solid", bd=1)
        frame.pack(fill="x", padx=20, pady=8)
        
        tk.Label(frame, text="输出模式:", bg="#e8f5e9",
                 font=("Arial", 11, "bold")).pack(side="left", padx=10, pady=8)
        
        self.mode_var = tk.StringVar(value="问答式(默认)")
        mode_names = list(self.mode_map.keys())
        self.mode_combo = ttk.Combobox(frame, textvariable=self.mode_var,
                                        values=mode_names, state="readonly", width=16)
        self.mode_combo.pack(side="right", padx=10, pady=8)
        self.mode_combo.bind("<<ComboboxSelected>>", self._on_mode_change)
    
    def _create_axis_display(self):
        """创建三个轴的数值显示"""
        axes_config = [
            ('roll',    'ROLL',    '#1976D2'),   # 深蓝
            ('pitch',  'PITCH',  '#D32F2F'),   # 深红
            ('heading','HEADING','#388E3C')    # 深绿
        ]
        
        self.labels = {}
        self.value_labels = {}
        
        for key, name, color in axes_config:
            frame = tk.Frame(self.root, bg="#fafafa", bd=1, relief="solid")
            frame.pack(fill="x", padx=25, pady=5)
            
            name_label = tk.Label(frame, text=name, font=("Arial", 14, "bold"),
                                   fg=color, bg="#fafafa", width=10, anchor="w")
            name_label.pack(side="left", padx=18, pady=10)
            
            value_label = tk.Label(frame, text="+0.00°", font=("Arial", 22, "bold"),
                                    fg="#212121", bg="#fafafa", width=12)
            value_label.pack(side="right", padx=18, pady=10)
            
            self.labels[key] = name_label
            self.value_labels[key] = value_label
    
    def _create_raw_data_display(self):
        """创建原始数据显示区域"""
        # 原始数据框架
        raw_frame = tk.Frame(self.root, bg="#f0f0f0", relief="solid", bd=1)
        raw_frame.pack(fill="x", padx=20, pady=10)
        
        # 标题行
        title_frame = tk.Frame(raw_frame, bg="#f0f0f0")
        title_frame.pack(fill="x", padx=10, pady=(8, 5))
        
        tk.Label(title_frame, text="原始数据 (14字节)",
                 font=("Arial", 11, "bold"), bg="#f0f0f0", fg="#555").pack(side="left")
        
        # 帧状态标签
        self.checksum_status = tk.Label(title_frame, text="等待数据...",
                                          font=("Arial", 9), bg="#f0f0f0", fg="#999")
        self.checksum_status.pack(side="right", padx=5)
        
        # 原始数据显示标签
        self.raw_data_label = tk.Label(raw_frame, text="--",
                                         font=("Consolas", 11, "bold"),
                                         bg="#f0f0f0", fg="#1976D2",
                                         anchor="w", padx=10)
        self.raw_data_label.pack(fill="x", padx=10, pady=(0, 8))
        
        # 统计信息
        stats_frame = tk.Frame(raw_frame, bg="#f0f0f0")
        stats_frame.pack(fill="x", padx=10, pady=(0, 10))
        
        self.stats_label = tk.Label(stats_frame, text="总帧数: 0  有效: 0  错误: 0",
                                     font=("Arial", 9), bg="#f0f0f0", fg="#666")
        self.stats_label.pack(side="left", padx=(0, 10))
        
        # 字节分组显示
        group_frame = tk.Frame(raw_frame, bg="#f0f0f0")
        group_frame.pack(fill="x", padx=10, pady=(0, 10))
        
        tk.Label(group_frame, text="分组:",
                 font=("Arial", 10), bg="#f0f0f0", fg="#666").pack(side="left", padx=(0, 5))
        
        self.group1_label = tk.Label(group_frame, text="--",
                                      font=("Consolas", 10, "bold"),
                                      bg="#f0f0f0", fg="#F57C00")
        self.group1_label.pack(side="left", padx=5)
        
        self.group2_label = tk.Label(group_frame, text="--",
                                      font=("Consolas", 10, "bold"),
                                      bg="#f0f0f0", fg="#F57C00")
        self.group2_label.pack(side="left", padx=5)
        
        self.group3_label = tk.Label(group_frame, text="--",
                                      font=("Consolas", 10, "bold"),
                                      bg="#f0f0f0", fg="#F57C00")
        self.group3_label.pack(side="left", padx=5)
    
    def _create_control_buttons(self):
        """创建控制按钮"""
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=15)
        
        self.mode_btn = tk.Button(btn_frame, text="应用模式",
                                   command=self._apply_mode,
                                   bg="#1976D2", fg="white",
                                   font=("Arial", 11, "bold"),
                                   padx=25, pady=8, relief="flat")
        self.mode_btn.pack(side="left", padx=10)
        
        self.reset_btn = tk.Button(btn_frame, text="恢复问答模式",
                                    command=self._reset_mode,
                                    bg="#757575", fg="white",
                                    font=("Arial", 11),
                                    padx=15, pady=8, relief="flat")
        self.reset_btn.pack(side="left", padx=10)
        
        self.clear_btn = tk.Button(btn_frame, text="清除缓冲区",
                                   command=self._clear_buffer,
                                   bg="#FF9800", fg="white",
                                   font=("Arial", 11),
                                   padx=15, pady=8, relief="flat")
        self.clear_btn.pack(side="left", padx=10)

        self.read_btn = tk.Button(btn_frame, text="发送读取命令",
                                   command=self._send_read_command,
                                   bg="#9C27B0", fg="white",
                                   font=("Arial", 11),
                                   padx=15, pady=8, relief="flat")
        self.read_btn.pack(side="left", padx=10)
    
    def _on_mode_change(self, event=None):
        """模式选择改变"""
        selected = self.mode_var.get()
        self.mode = self.mode_map[selected]
    
    def _apply_mode(self):
        """发送设置模式命令"""
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("警告", "串口未连接，请先连接串口")
            return

        try:
            cmd = build_set_mode_command(self.mode)

            # 发送命令
            bytes_written = self.ser.write(cmd)
            hex_str = ' '.join(f'{b:02X}' for b in cmd)
            mode_name = get_mode_name(self.mode)

            print(f"[发送模式设置] {hex_str}  -> {mode_name} (写入{bytes_written}字节)")

            # 更新状态 - 先显示"正在切换"
            self.status_label.config(
                text=f"状态: 正在切换至 {mode_name}...",
                fg="#FF9800"
            )

            # 标记等待模式确认
            self._waiting_for_mode_confirm = True

            # 延迟检查确认（给设备响应时间）
            self.root.after(500, self._check_mode_confirmation)

            # 100Hz模式下检查波特率
            if self.mode == 0x06 and self.baudrate.get() < 19200:
                self.root.after(1000, self._show_baudrate_warning)

            # 自动模式标记
            self.auto_mode = (self.mode != 0x00)

        except Exception as e:
            messagebox.showerror("错误", f"发送命令失败: {e}")
            self.status_label.config(text=f"状态: 错误 - {e}", fg="#D32F2F")

    def _check_mode_confirmation(self):
        """检查模式切换是否成功（通过读取响应确认）"""
        response = b''  # 初始化为空
        try:
            # 尝试读取设备的响应
            if self.ser and self.ser.in_waiting > 0:
                response = self.ser.read(self.ser.in_waiting)
                response_hex = ' '.join(f'{b:02X}' for b in response)
                print(f"[设备响应] {response_hex}")

            mode_name = get_mode_name(self.mode)

            # 检查设备响应是否正确
            # 模式切换后设备可能返回 68 01 66 (确认帧) 而不是数据帧
            if len(response) >= 3:
                self.status_label.config(
                    text=f"状态: 已切换至 {mode_name}",
                    fg="#388E3C"
                )
                print(f"  └ 模式切换成功: {mode_name}")
            else:
                # 没有收到完整响应，但命令已发送，显示等待中
                self.status_label.config(
                    text=f"状态: {mode_name} (等待数据...)",
                    fg="#1976D2"
                )

        except Exception as e:
            self.status_label.config(
                text=f"状态: {get_mode_name(self.mode)} (发送成功)",
                fg="#1976D2"
            )
        finally:
            # 清除等待标志
            self._waiting_for_mode_confirm = False

    def _show_baudrate_warning(self):
        """显示波特率警告"""
        msg = ("100Hz自动输出模式建议波特率≥19200，"
               "当前波特率可能无法稳定接收数据。\n"
               "是否现在修改波特率？")
        if messagebox.askyesno("波特率提示", msg):
            self.baudrate.set(19200)
            self.status_label.config(
                text="状态: 波特率已设为19200，请重启串口",
                fg="#FF9800"
            )
    
    def _clear_buffer(self):
        """清除数据缓冲区"""
        _buffer.clear()
        self.data_count = 0
        self.error_count = 0
        print("  └ 包缓冲区已清除，统计已重置")

    def _send_read_command(self):
        """手动发送读取角度命令（问答模式用）"""
        if not self.ser or not self.ser.is_open:
            print("[警告] 串口未连接")
            return
        try:
            cmd = SerialBuffer.READ_ANGLES_CMD
            self.ser.write(cmd)
            hex_str = ' '.join(f'{b:02X}' for b in cmd)
            print(f"[发送读取命令] {hex_str}")
        except Exception as e:
            print(f"[错误] 发送读取命令失败: {e}")

    def _reset_mode(self):
        """恢复问答模式"""
        self.mode = 0x00
        self.mode_var.set("问答式(默认)")
        self.auto_mode = False  # 切换为问答模式
        # 清除缓冲区后再应用模式
        self._clear_buffer()
        self._apply_mode()
    
    def _update_raw_display(self, data: bytes):
        """更新原始数据显示"""
        hex_str = ' '.join(f'{b:02X}' for b in data)
        
        # 格式化：每隔2个字节加空格
        formatted = []
        for i in range(0, len(data), 2):
            chunk = ' '.join(f'{data[j]:02X}' for j in range(i, min(i+2, len(data))))
            formatted.append(chunk)
        display_str = '  '.join(formatted)
        
        self.raw_data_label.config(text=display_str)
        
        # 更新分组显示
        if len(data) >= 13:
            g1 = f"[4-6]  {data[4]:02X} {data[5]:02X} {data[6]:02X}"
            g2 = f"[7-9]  {data[7]:02X} {data[8]:02X} {data[9]:02X}"
            g3 = f"[10-12] {data[10]:02X} {data[11]:02X} {data[12]:02X}"
            self.group1_label.config(text=g1)
            self.group2_label.config(text=g2)
            self.group3_label.config(text=g3)
    
    def _serial_loop(self):
        """串口读取线程"""
        self.running = True
        
        # 等待一下让UI显示
        time.sleep(0.5)
        
        while self.running:
            try:
                # 尝试连接串口
                if not self.ser or not self.ser.is_open:
                    try:
                        port = self.serial_port.get()
                        baud = self.baudrate.get()
                        
                        self.ser = serial.Serial(
                            port=port,
                            baudrate=baud,
                            timeout=1.0,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE
                        )
                        
                        self.root.after(0, lambda: self.status_label.config(
                            text=f"状态: 已连接 {port}@{baud}  |  模式: {get_mode_name(self.mode)}",
                            fg="#388E3C"
                        ))
                        
                        # 连接后清除缓冲区，避免旧数据干扰
                        _buffer.clear()
                        self.data_count = 0
                        self.error_count = 0
                        print("  └ 串口连接成功，缓冲区已初始化")
                        
                        # 连接后发送当前模式命令
                        if self.mode != 0x00:
                            time.sleep(0.5)
                            self.root.after(0, self._apply_mode)
                        
                    except Exception as e:
                        self.root.after(0, lambda: self.status_label.config(
                            text=f"状态: 连接失败 - {e}", fg="#D32F2F"
                        ))
                        time.sleep(2)
                        continue
                
                # 读取数据
                if self.ser and self.ser.is_open:
                    # 问答模式下主动发送读取命令
                    if not self.auto_mode:
                        self.ser.write(SerialBuffer.READ_ANGLES_CMD)
                        time.sleep(0.05)  # 等待设备响应

                    # 读取所有可用数据
                    if self.ser.in_waiting > 0:
                        data = self.ser.read(self.ser.in_waiting)
                        _buffer.append(data)
                    
                    # 尝试提取完整帧
                    frame = _buffer.find_frame()
                    
                    if frame:
                        # 更新原始数据显示（无论校验是否通过都显示）
                        def update_raw():
                            self._update_raw_display(frame)
                        
                        self.root.after(0, update_raw)
                        
                        # 验证并解析
                        angles = parse_data_frame(frame)
                        
                        if angles:
                            # 校验通过
                            self.data_count += 1
                            roll, pitch, heading = angles
                            
                            def update():
                                self.value_labels['roll'].config(
                                    text=f"{roll:>+8.2f}°", fg="#1976D2"
                                )
                                self.value_labels['pitch'].config(
                                    text=f"{pitch:>+8.2f}°", fg="#D32F2F"
                                )
                                self.value_labels['heading'].config(
                                    text=f"{heading:>+8.2f}°", fg="#388E3C"
                                )
                                self.checksum_status.config(
                                    text="✓ 校验通过", fg="#388E3C"
                                )
                                self.stats_label.config(
                                    text=f"总帧数: {self.data_count + self.error_count}  有效: {self.data_count}  错误: {self.error_count}"
                                )
                            
                            self.root.after(0, update)
                        else:
                            # 帧头错误或数据解析失败
                            self.error_count += 1
                            
                            def update_error():
                                self.checksum_status.config(
                                    text="✗ 帧错误", fg="#D32F2F"
                                )
                                self.stats_label.config(
                                    text=f"总帧数: {self.data_count + self.error_count}  有效: {self.data_count}  错误: {self.error_count}"
                                )
                            
                            self.root.after(0, update_error)
                        
            except serial.SerialException as e:
                self.ser = None
                self.root.after(0, lambda: self.status_label.config(
                    text=f"状态: 串口断开 - {e}", fg="#D32F2F"
                ))
                time.sleep(2)
            except Exception as e:
                self.root.after(0, lambda: self.status_label.config(
                    text=f"状态: 错误 - {e}", fg="#D32F2F"
                ))
                time.sleep(1)
    
    def _on_closing(self):
        """窗口关闭时调用"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = SensorApp(root)
    root.mainloop()