"""
航向角锁定自动驾驶控制模块

结合卡尔曼滤波罗盘、PID控制器与差速驱动，实现航向角锁定自动修正功能

功能特性:
1. 启动时锁定当前航向角作为目标航向
2. 实时检测航向角偏差，使用PID控制器进行精确修正
3. 支持参数配置（PID参数、偏差阈值等）
4. 差速修正根据偏差大小动态调整
5. 集成航向角回环处理模块，解决 0°/360° 边界跳变问题

使用示例:
    from heading_lock_control import HeadingLockController

    controller = HeadingLockController(
        compass_port='/dev/ttyS0',
        base_speed=50,
        pid_kp=2.0,
        pid_ki=0.1,
        pid_kd=0.5
    )
    controller.start()
    controller.run_loop(duration=60)
"""

import sys
import time
import math
import threading
from typing import Optional, Tuple, Callable

sys.path.insert(0, __file__.rsplit('/', 1)[0] if '/' in __file__ else '.')

_root_path = __file__.rsplit('/', 1)[0] if '/' in __file__ else '.'

from Three_axis_angles.compass_kalman import CompassKalmanFilter, FilteredCompassData
from Three_axis_angles.ddm350b import DDM350B, OutputMode
from control_car.dual_motor_control import create_dual_motor_driver, DifferentialDrive
from heading_wrap import HeadingWrapReader

try:
    from Gps.gps import GPSReader, GPSPosition
    from Gps.gps_navigation_controller import GPSNavigationController
    GPS_AVAILABLE = True
except ImportError:
    GPS_AVAILABLE = False
    GPSReader = None
    GPSPosition = None
    GPSNavigationController = None


class PIDController:
    """
    增量式PID控制器

    用于航向角修正控制，输出为差速修正量
    """

    def __init__(
        self,
        kp: float = 1.5,
        ki: float = 0.01,
        kd: float = 1,
        output_min: float = -1.0,
        output_max: float = 1.0,
        deadband: float = 5,
        derivative_filter: float = 0.3
    ):
        """
        初始化PID控制器

        Args:
            kp: 比例系数
            ki: 积分系数
            kd: 微分系数
            output_min: 输出最小值
            output_max: 输出最大值
            deadband: 死区范围（在此范围内的误差不产生输出）
            derivative_filter: 微分滤波系数 (0-1)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.deadband = deadband
        self.derivative_filter = derivative_filter

        self._last_error: float = 0.0
        self._integral: float = 0.0
        self._last_output: float = 0.0
        self._filtered_derivative: float = 0.0
        self._initialized: bool = False
        self._first_run: bool = True

    def reset(self):
        """重置PID控制器状态"""
        self._last_error = 0.0
        self._integral = 0.0
        self._last_output = 0.0
        self._filtered_derivative = 0.0
        self._initialized = False
        self._first_run = True

    def compute(self, error: float, dt: float = None) -> float:
        """
        计算PID输出

        Args:
            error: 当前误差（目标值 - 实际值）
            dt: 控制周期(秒)，如果为None则自动计算

        Returns:
            PID控制输出值 (output_min ~ output_max)
        """
        if not self._initialized:
            self._last_error = error
            self._initialized = True
            return 0.0

        if dt is None:
            dt = 0.1

        if self._first_run:
            self._first_run = False
            self._last_error = error
            return 0.0

        if abs(error) < self.deadband:
            return 0.0

        p_term = self.kp * error

        self._integral += error * dt
        self._integral = max(-50, min(50, self._integral))
        i_term = self.ki * self._integral

        raw_derivative = (error - self._last_error) / dt
        self._filtered_derivative = (
            self.derivative_filter * raw_derivative +
            (1 - self.derivative_filter) * self._filtered_derivative
        )
        d_term = self.kd * self._filtered_derivative

        output = p_term + i_term + d_term

        output = max(self.output_min, min(self.output_max, output))

        self._last_error = error

        return output

    def set_tuning(self, kp: float = None, ki: float = None, kd: float = None):
        """动态调整PID参数"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

    def get_state(self) -> dict:
        """获取PID状态信息"""
        return {
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd,
            'integral': self._integral,
            'last_error': self._last_error,
            'filtered_derivative': self._filtered_derivative
        }


class HeadingLockController:
    """
    航向角锁定控制器

    通过罗盘获取航向角，锁定目标航向，使用PID控制器进行精确差速修正
    """

    def __init__(
        self,
        compass_port: str = '/dev/ttyS0',
        base_speed: int = 80,
        deviation_threshold: float = 5.0,
        pid_kp: float = 2.0,
        pid_ki: float = 0.1,
        pid_kd: float = 0.5,
        process_noise: float = 0.001,
        measurement_noise: float = 0.1,
        update_interval: float = 0.02,
        min_turn_strength: float = 0.05,
        max_turn_strength: float = 0.2,
        compass_mode: OutputMode = OutputMode.AUTO_50HZ,
        use_heading_wrap: bool = True
    ):
        """
        初始化航向角锁定控制器

        Args:
            compass_port: 罗盘串口设备路径
            base_speed: 直行基础速度 (0-100)
                        注意: 值越大速度越慢
            deviation_threshold: 偏差死区阈值(度)，低于此值不进行修正
            pid_kp: PID比例系数
            pid_ki: PID积分系数
            pid_kd: PID微分系数
            process_noise: 卡尔曼滤波过程噪声
            measurement_noise: 卡尔曼滤波测量噪声
            update_interval: 控制周期(秒)，默认0.02s匹配50Hz输出
            min_turn_strength: 最小转弯强度 (0.0 ~ 1.0)
            max_turn_strength: 最大转弯强度 (0.0 ~ 1.0)
            compass_mode: 罗盘输出模式，默认AUTO_50HZ (50Hz)
            use_heading_wrap: 是否使用航向角回环处理（解决0°/360°边界跳变）
        """
        self.compass_port = compass_port
        self.base_speed = base_speed
        self.deviation_threshold = deviation_threshold
        self.update_interval = update_interval
        self.min_turn_strength = min_turn_strength
        self.max_turn_strength = max_turn_strength
        self.compass_mode = compass_mode
        self.use_heading_wrap = use_heading_wrap

        self._pid = PIDController(
            kp=pid_kp,
            ki=pid_ki,
            kd=pid_kd,
            output_min=-1.0,
            output_max=1.0,
            deadband=deviation_threshold,
            derivative_filter=0.3
        )

        self._heading_wrap_reader = None  # 回环读取器
        self._compass = None
        self._kalman_filter = None
        self._driver = None
        self._target_heading: float = None
        self._target_continuous_heading: float = None  # 连续目标航向
        self._is_running = False
        self._last_update_time: float = None
        self._iteration_count: int = 0
        self._debug: bool = False
        self._last_left_speed: int = 0
        self._last_right_speed: int = 0

        self._stats = {
            'max_error': 0.0,
            'avg_error': 0.0,
            'correction_count': 0
        }

        self._gps_enabled = False
        self._gps_navigation: Optional[GPSNavigationController] = None
        self._target_lat: Optional[float] = None
        self._target_lon: Optional[float] = None
        self._gps_bearing: float = 0.0
        self._gps_distance: float = float('inf')
        self._calibration_heading: float = 0.0

    def _init_gps_navigation(self, gps_port: str = '/dev/ttyS1', gps_baudrate: int = 38400) -> bool:
        """
        初始化GPS导航控制器（使用gps_navigation_controller.py）

        Args:
            gps_port: GPS串口路径
            gps_baudrate: GPS波特率

        Returns:
            是否成功
        """
        if not GPS_AVAILABLE:
            print("[GPS] GPSNavigationController不可用，请检查 Gps/gps_navigation_controller.py 是否存在")
            return False

        try:
            self._gps_navigation = GPSNavigationController(
                target_lat=self._target_lat or 0.0,
                target_lon=self._target_lon or 0.0,
                compass_port=self.compass_port,
                gps_port=gps_port,
                gps_baudrate=gps_baudrate,
                calibration_speed=self.base_speed
            )
            print(f"[GPS] GPSNavigationController已初始化")
            return True

        except Exception as e:
            print(f"[GPS] GPSNavigationController初始化失败: {e}")
            return False

    def _calibrate_with_gps(self, duration: float = 2.0) -> bool:
        """
        使用GPS导航控制器校准车头方向

        Args:
            duration: 前进校准时间(秒)

        Returns:
            是否成功
        """
        if not self._gps_navigation:
            return False

        print(f"[GPS] 开始校准车头方向 (前进{duration}秒)...")

        if not self._gps_navigation._calibrate_heading():
            print("[GPS] 车头校准失败")
            return False

        self._calibration_heading = self._gps_navigation._calibration_heading
        print(f"[GPS] 车头方向已校准: {self._calibration_heading:.1f}°")
        return True

    def update_gps_navigation(self):
        """更新GPS导航状态（从GPSNavigationController获取航向角）"""
        if not self._gps_navigation or not self._gps_navigation._is_initialized:
            return

        self._gps_navigation._update_navigation()
        state = self._gps_navigation._state

        self._gps_bearing = state.bearing_angle
        self._gps_distance = state.distance_to_target
        self._target_heading = state.target_heading

    def set_gps_target(self, lat: float, lon: float):
        """
        设置GPS目标点

        Args:
            lat: 目标纬度
            lon: 目标经度
        """
        self._target_lat = lat
        self._target_lon = lon
        if self._gps_navigation:
            self._gps_navigation.set_target(lat, lon)
        print(f"[GPS] 目标点已设置: ({lat:.6f}, {lon:.6f})")

    def get_gps_info(self) -> Tuple[float, float, float]:
        """
        获取GPS导航信息

        Returns:
            (方位角, 距离, 目标航向角)
        """
        return (self._gps_bearing, self._gps_distance, self._target_heading)

    def confirm_target_heading(self, bearing: float, distance: float):
        """
        确认并显示目标航向角

        Args:
            bearing: 方位角(度)
            distance: 距离(米)
        """
        direction_names = ["北", "东北", "东", "东南", "南", "西南", "西", "西北"]
        index = int((bearing + 22.5) / 45) % 8

        distance_str = f"{distance:.1f}" if distance != float('inf') else "计算中..."

        print("\n" + "=" * 60)
        print("  目标航线角确认")
        print("=" * 60)
        print(f"  方位角: {bearing:.1f}° ({direction_names[index]})")
        print(f"  距离:   {distance_str} 米")
        print(f"  目标:   ({self._target_lat:.6f}, {self._target_lon:.6f})")
        print("=" * 60)

    def _init_compass(self) -> bool:
        """初始化罗盘连接，设置为50Hz自动输出模式"""
        try:
            if self.use_heading_wrap:
                self._heading_wrap_reader = HeadingWrapReader(
                    port=self.compass_port,
                    process_noise=0.001,
                    measurement_noise=0.1,
                    compass_mode=self.compass_mode
                )
                if not self._heading_wrap_reader.start():
                    return False
                print(f"[罗盘] 已连接到 {self.compass_port}，模式: {self.compass_mode.name if hasattr(self.compass_mode, 'name') else str(self.compass_mode)}")
                print("[罗盘] 航向角回环处理已启用")
                return True

            self._compass = DDM350B(self.compass_port)
            if not self._compass.connect():
                print(f"[错误] 无法连接到罗盘 {self.compass_port}")
                return False

            self._kalman_filter = CompassKalmanFilter(
                process_noise=0.001,
                measurement_noise=0.1
            )

            if not self._compass.set_mode(self.compass_mode):
                print(f"[警告] 无法设置罗盘模式为 {self.compass_mode.name}，使用默认模式")

            mode_name = self.compass_mode.name if hasattr(self.compass_mode, 'name') else str(self.compass_mode)
            print(f"[罗盘] 已连接到 {self.compass_port}，模式: {mode_name}")
            return True
        except Exception as e:
            print(f"[错误] 罗盘初始化失败: {e}")
            return False

    def _init_motor_driver(self) -> bool:
        """初始化电机驱动"""
        try:
            self._driver = create_dual_motor_driver(self.base_speed)
            print(f"[电机] 基础速度设置: {self.base_speed}%")
            return True
        except Exception as e:
            print(f"[错误] 电机驱动初始化失败: {e}")
            return False

    def _calibrate_target_heading(self) -> bool:
        """校准目标航向角"""
        print("[校准] 正在校准目标航向角...")
        samples = []
        samples_continuous = []

        if self.use_heading_wrap and self._heading_wrap_reader:
            for i in range(10):
                data = self._heading_wrap_reader.update()
                if data:
                    samples.append(data.wrapped_heading)
                    samples_continuous.append(data.continuous_heading)
                time.sleep(0.1)

            if len(samples) < 5:
                print("[错误] 罗盘数据采集不足，校准失败")
                return False

            self._target_heading = sum(samples) / len(samples)
            self._target_continuous_heading = sum(samples_continuous) / len(samples_continuous)
            print(f"[校准] 目标航向角已锁定: {self._target_heading:.1f}° (连续: {self._target_continuous_heading:.1f}°)")
        else:
            for i in range(10):
                raw_data = self._compass.read()
                if raw_data:
                    filtered = self._kalman_filter.update(raw_data)
                    samples.append(filtered.heading)
                time.sleep(0.1)

            if len(samples) < 5:
                print("[错误] 罗盘数据采集不足，校准失败")
                return False

            self._target_heading = sum(samples) / len(samples)
            self._target_continuous_heading = self._target_heading
            print(f"[校准] 目标航向角已锁定: {self._target_heading:.1f}°")
        return True

    def _calculate_heading_error(self, current_heading: float, continuous_heading: float = None) -> float:
        """
        计算航向角偏差

        Args:
            current_heading: 当前航向角 (0-360°)
            continuous_heading: 连续航向角（可超过360°），如果提供则使用连续角度计算
        """
        if self._target_heading is None:
            return 0.0

        if continuous_heading is not None and self._target_continuous_heading is not None:
            error = continuous_heading - self._target_continuous_heading
        else:
            error = current_heading - self._target_heading

        while error > 180:
            error -= 360
        while error < -180:
            error += 360

        return error

    def _apply_pid_correction(self, error: float):
        """
        使用PID控制器应用差速修正

        修正逻辑（差速转弯）:
        - error > 0 (航向偏右) → 需要左转 → 右轮减速
        - error < 0 (航向偏左) → 需要右转 → 左轮减速

        Args:
            error: 航向角偏差 (-180° ~ 180°)
        """
        pid_output = self._pid.compute(error, self.update_interval)

        if abs(pid_output) < 0.01:
            self._driver.forward(self.base_speed)
            self._last_left_speed = self.base_speed
            self._last_right_speed = self.base_speed
            return

        left_correction = 0.0
        right_correction = 0.0

        if error > 0:
            right_correction = min(abs(pid_output), self.max_turn_strength)
        else:
            left_correction = min(abs(pid_output), self.max_turn_strength)

        left_factor = 1.0 - left_correction
        right_factor = 1.0 - right_correction

        left_speed = int(self.base_speed * left_factor)
        right_speed = int(self.base_speed * right_factor)

        self._driver.left_motor.forward(left_speed)
        self._driver.right_motor.forward(right_speed)

        self._last_left_speed = left_speed
        self._last_right_speed = right_speed

        if abs(error) > self._stats['max_error']:
            self._stats['max_error'] = abs(error)

    def start(self, calibrate: bool = True) -> bool:
        """启动航向角锁定控制"""
        if not self._init_compass():
            return False

        if not self._init_motor_driver():
            self._compass.disconnect()
            return False

        if calibrate:
            if not self._calibrate_target_heading():
                self._compass.disconnect()
                return False

        self._pid.reset()
        self._is_running = True
        self._last_update_time = time.time()
        print("[启动] 航向角锁定控制已启动 (Ctrl+C 停止)")
        print(f"[PID] Kp={self._pid.kp}, Ki={self._pid.ki}, Kd={self._pid.kd}")
        return True

    def _format_info_panel(
        self,
        current_heading: float,
        error: float,
        pid_state: dict,
        status: str,
        elapsed_time: float,
        continuous_heading: float = None
    ) -> str:
        """格式化信息面板"""
        panel_width = 60

        error_bar = self._make_error_bar(error)

        left_speed_display = self._last_left_speed if self._debug else 0
        right_speed_display = self._last_right_speed if self._debug else 0

        uptime = self._format_uptime(elapsed_time)
        status_icon = "✓" if status == "直行" else "⟲"

        wrap_info = ""
        if self.use_heading_wrap and continuous_heading is not None:
            wrap_info = f"(连续: {continuous_heading:.1f}°)"

        panel = f"""
╔══════════════════════════════════════════════════════════════════════╗
║                         航向角锁定控制系统                              ║
╠══════════════════════════════════════════════════════════════════════╣
║  目标航向: {self._target_heading:>6.1f}°                                   运行时间: {uptime}     ║
║  当前航向: {current_heading:>6.1f}° {wrap_info:<25}  状态: {status_icon} {status:<12}║
║  航向偏差: {error:>+6.1f}°  {error_bar}                            ║
╠══════════════════════════════════════════════════════════════════════╣
║  PID 控制输出                                                         ║
║    ├─ Kp: {pid_state['kp']:>5.2f}  │  P输出: {pid_state['kp']*error:>+7.2f}                     ║
║    ├─ Ki: {pid_state['ki']:>5.2f}  │  I输出: {pid_state['ki']*pid_state['integral']:>+7.2f}                     ║
║    └─ Kd: {pid_state['kd']:>5.2f}  │  D输出: {pid_state['kd']*pid_state['filtered_derivative']:>+7.2f}                     ║
╠══════════════════════════════════════════════════════════════════════╣
║  电机控制                                                             ║
║    ├─ 基础速度: {self.base_speed:>3d}%                                          ║
║    └─ 左右速度: 左 {left_speed_display:>3d}%  右 {right_speed_display:>3d}%                      ║
╠══════════════════════════════════════════════════════════════════════╣
║  统计信息                                                             ║
║    ├─ 最大偏差: {self._stats['max_error']:>5.1f}°                                     ║
║    ├─ 平均偏差: {self._stats['avg_error']:>5.1f}°                                      ║
║    └─ 修正次数: {self._stats['correction_count']:>5d} / {self._iteration_count:<5d}                                  ║
╚══════════════════════════════════════════════════════════════════════╝"""
        return panel

    def _make_error_bar(self, error: float, bar_width: int = 15) -> str:
        """创建偏差可视化条"""
        max_error = 30.0
        normalized = max(-1, min(1, error / max_error))
        bar_len = int(abs(normalized) * bar_width)

        if error >= 0:
            bar = "─" * bar_len + "▶"
            bar = bar.ljust(bar_width + 1)
            bar = f"{bar} +{error:.1f}°"
        else:
            bar = "◀" + "─" * bar_len
            bar = bar.rjust(bar_width + 1)
            bar = f"{bar} {error:.1f}°"

        return bar[:bar_width + 6]

    def _format_uptime(self, seconds: float) -> str:
        """格式化运行时间"""
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)
        if hours > 0:
            return f"{hours:02d}:{minutes:02d}:{secs:02d}"
        return f"{minutes:02d}:{secs:02d}"

    def _update_display(self, current_heading: float, error: float, elapsed_time: float, continuous_heading: float = None):
        """更新终端显示（覆盖式刷新）"""
        pid_state = self._pid.get_state()
        status = "直行" if abs(error) < self.deviation_threshold else "修正中"

        panel = self._format_info_panel(current_heading, error, pid_state, status, elapsed_time, continuous_heading)

        sys.stdout.write("\033[2J\033[H")
        sys.stdout.write(panel)
        sys.stdout.flush()

    def run_loop(self, duration: float = None):
        """运行控制循环"""
        if not self._is_running:
            print("[错误] 控制器未启动，请先调用 start()")
            return

        print("[运行] 开始航向角锁定控制循环 (每0.1秒刷新显示)")
        print("[提示] 按 Ctrl+C 可随时停止\n")
        start_time = time.time()
        error_sum = 0.0
        self._iteration_count = 0
        last_display_time = 0.0
        display_interval = 0.1

        try:
            while self._is_running:
                loop_start = time.time()

                # 更新GPS导航状态（如果有）
                if self._gps_enabled and self._gps_navigation:
                    self.update_gps_navigation()
                    # GPS模式下，从GPSNavigationController获取罗盘数据
                    if self._gps_navigation._heading_lock:
                        inner_heading_lock = self._gps_navigation._heading_lock
                        current_heading = inner_heading_lock.get_current_heading()
                        if current_heading is None:
                            time.sleep(0.05)
                            continue
                        continuous_heading = inner_heading_lock.get_continuous_heading()
                    else:
                        current_heading = None
                        continuous_heading = None
                elif self.use_heading_wrap and self._heading_wrap_reader:
                    data = self._heading_wrap_reader.update()
                    if data is None:
                        time.sleep(0.05)
                        continue
                    current_heading = data.wrapped_heading
                    continuous_heading = data.continuous_heading
                else:
                    raw_data = self._compass.read()
                    if raw_data is None:
                        time.sleep(0.1)
                        continue
                    filtered = self._kalman_filter.update(raw_data)
                    current_heading = filtered.heading
                    continuous_heading = None

                error = self._calculate_heading_error(current_heading, continuous_heading)
                error_sum += abs(error)
                self._iteration_count += 1
                self._stats['avg_error'] = error_sum / self._iteration_count

                if abs(error) >= self.deviation_threshold:
                    self._stats['correction_count'] += 1

                # 在GPS模式下使用内部HeadingLockController的驱动
                if self._gps_enabled and self._gps_navigation and self._gps_navigation._heading_lock:
                    self._gps_navigation._heading_lock._apply_pid_correction(error)
                    self._last_left_speed = self._gps_navigation._heading_lock._last_left_speed
                    self._last_right_speed = self._gps_navigation._heading_lock._last_right_speed
                else:
                    self._apply_pid_correction(error)

                loop_end_time = time.time()
                elapsed = loop_end_time - start_time

                if loop_end_time - last_display_time >= display_interval:
                    self._update_display(current_heading, error, elapsed, continuous_heading)
                    last_display_time = loop_end_time

                if duration and elapsed >= duration:
                    break

                remaining = self.update_interval - (time.time() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)

        except KeyboardInterrupt:
            print("\n[停止] 用户中断")

        finally:
            sys.stdout.write("\n")
            self.stop()

    def stop(self):
        """停止控制"""
        self._is_running = False
        if self._driver:
            self._driver.stop()
            print("[停止] 电机已停止")
        if self._heading_wrap_reader:
            self._heading_wrap_reader.stop()
            self._heading_wrap_reader = None
        if self._compass:
            self._compass.disconnect()
            print("[停止] 罗盘已断开")
        if self._gps_navigation:
            self._gps_navigation.stop()
            self._gps_navigation = None
            self._gps_enabled = False
            print("[停止] GPS已断开")
        print("[停止] 航向角锁定控制已关闭")

        if self._iteration_count > 0:
            print(f"[统计] 最大偏差: {self._stats['max_error']:.1f}° | "
                  f"平均偏差: {self._stats['avg_error']:.1f}° | "
                  f"修正次数: {self._stats['correction_count']}/{self._iteration_count}")

    def set_target_heading(self, heading: float):
        """手动设置目标航向角"""
        self._target_heading = heading % 360
        self._pid.reset()
        print(f"[设置] 目标航向角已更新: {self._target_heading:.1f}°")

    def set_pid_tuning(self, kp: float = None, ki: float = None, kd: float = None):
        """动态调整PID参数"""
        self._pid.set_tuning(kp, ki, kd)
        print(f"[PID] 参数已更新: Kp={self._pid.kp}, Ki={self._pid.ki}, Kd={self._pid.kd}")

    def get_current_heading(self) -> float:
        """获取当前航向角 (0-360°)"""
        if self.use_heading_wrap and self._heading_wrap_reader:
            data = self._heading_wrap_reader.get_full_data()
            if data:
                return data.wrapped_heading
        elif self._compass:
            raw_data = self._compass.read()
            if raw_data:
                filtered = self._kalman_filter.update(raw_data)
                return filtered.heading
        return None

    def get_continuous_heading(self) -> float:
        """获取当前连续航向角（可超过360°）"""
        if self.use_heading_wrap and self._heading_wrap_reader:
            return self._heading_wrap_reader.get_heading()
        return self.get_current_heading()

    def get_pid_state(self) -> dict:
        """获取PID状态"""
        return self._pid.get_state()

    def enable_debug(self):
        """启用调试模式，显示电机速度"""
        self._debug = True
        print("[调试] 调试模式已启用")

    def disable_debug(self):
        """禁用调试模式"""
        self._debug = False
        print("[调试] 调试模式已禁用")

    def __enter__(self):
        """支持 with 语句"""
        self.start()
        return self

    def __exit__(self, *args):
        """退出 with 语句时停止"""
        self.stop()


def heading_lock_demo(duration: float = 60):
    """航向角锁定演示"""
    print("\n" + "=" * 60)
    print("  航向角锁定自动驾驶演示 (PID控制 + 50Hz)")
    print("=" * 60)

    controller = HeadingLockController(
        compass_port='/dev/ttyS0',
        base_speed=80,
        deviation_threshold=20.0,
        pid_kp=2.0,
        pid_ki=0.1,
        pid_kd=0.5,
        update_interval=0.02,
        compass_mode=OutputMode.AUTO_50HZ
    )

    if not controller.start(calibrate=True):
        print("\n初始化失败，请检查硬件连接")
        return

    print(f"\n[演示] 将运行 {duration} 秒...")
    print("[提示] 尝试改变载体方向，系统会自动进行PID差速修正")
    print("[提示] 按 Ctrl+C 可随时停止\n")

    controller.run_loop(duration=duration)


def pid_tuning_helper():
    """
    PID参数调试助手

    逐步调整PID参数，帮助找到最佳值
    """
    print("\n" + "=" * 60)
    print("  PID参数调试助手")
    print("=" * 60)

    controller = HeadingLockController(
        compass_port='/dev/ttyS0',
        base_speed=80,
        deviation_threshold=20.0,
        pid_kp=1.0,
        pid_ki=0.0,
        pid_kd=0.0,
        update_interval=0.1
    )

    if not controller.start(calibrate=True):
        return

    print("\n[调试] 开始PID参数调试...")
    print("  Kp: 调整响应速度，太大容易震荡")
    print("  Ki: 消除稳态误差，太大容易超调")
    print("  Kd: 抑制超前调节，太大容易抖动")
    print("\n可用命令:")
    print("  p <值> - 调整Kp")
    print("  i <值> - 调整Ki")
    print("  d <值> - 调整Kd")
    print("  s      - 显示当前PID状态")
    print("  q      - 退出\n")

    controller.run_loop(duration=300)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='航向角锁定自动驾驶控制 (PID + GPS)')
    parser.add_argument('-d', '--duration', type=float, default=60,
                        help='运行时长(秒)，默认20秒')
    parser.add_argument('-p', '--port', type=str, default='/dev/ttyS0',
                        help='罗盘串口路径，默认 /dev/ttyS0')
    parser.add_argument('-s', '--speed', type=int, default=75,
                        help='基础速度 (0-100)，默认50')
    parser.add_argument('-t', '--threshold', type=float, default=20.0,
                        help='偏差死区阈值(度)，默认3度')
    parser.add_argument('--kp', type=float, default=1.0,
                        help='PID比例系数，默认2.0')
    parser.add_argument('--ki', type=float, default=0.05,
                        help='PID积分系数，默认0.1')
    parser.add_argument('--kd', type=float, default=0.8,
                        help='PID微分系数，默认0.5')
    parser.add_argument('-i', '--interactive', action='store_true',
                        help='启动PID调试模式')
    parser.add_argument('--mode', type=str, default='auto_50hz',
                        choices=['polling', 'auto_5hz', 'auto_15hz', 'auto_25hz',
                                'auto_35hz', 'auto_50hz', 'auto_100hz'],
                        help='罗盘输出模式，默认auto_50hz')
    parser.add_argument('--no-wrap', action='store_true',
                        help='禁用航向角回环处理（默认启用）')

    # GPS 相关参数
    parser.add_argument('--gps', action='store_true', help='启用GPS导航模式')
    parser.add_argument('--gps-port', type=str, default='/dev/ttyS1', help='GPS串口路径')
    parser.add_argument('--gps-baudrate', type=int, default=38400, help='GPS波特率')
    parser.add_argument('--target-lat', type=float, help='目标纬度')
    parser.add_argument('--target-lon', type=float, help='目标经度')
    parser.add_argument('--confirm', action='store_true', default=True,
                        help='确认目标航向角后启动（默认启用）')

    args = parser.parse_args()

    mode_map = {
        'polling': OutputMode.POLLING,
        'auto_5hz': OutputMode.AUTO_5HZ,
        'auto_15hz': OutputMode.AUTO_15HZ,
        'auto_25hz': OutputMode.AUTO_25HZ,
        'auto_35hz': OutputMode.AUTO_35HZ,
        'auto_50hz': OutputMode.AUTO_50HZ,
        'auto_100hz': OutputMode.AUTO_100HZ,
    }

    interval_map = {
        'polling': 0.1,
        'auto_5hz': 0.2,
        'auto_15hz': 0.067,
        'auto_25hz': 0.04,
        'auto_35hz': 0.029,
        'auto_50hz': 0.02,
        'auto_100hz': 0.01,
    }

    if args.interactive:
        pid_tuning_helper()
    else:
        controller = HeadingLockController(
            compass_port=args.port,
            base_speed=args.speed,
            deviation_threshold=args.threshold,
            pid_kp=args.kp,
            pid_ki=args.ki,
            pid_kd=args.kd,
            compass_mode=mode_map[args.mode],
            update_interval=interval_map[args.mode],
            use_heading_wrap=not args.no_wrap
        )

        # GPS 导航模式
        if args.gps:
            if args.target_lat is None or args.target_lon is None:
                print("[错误] GPS模式需要指定 --target-lat 和 --target-lon")
                sys.exit(1)

            if not GPS_AVAILABLE:
                print("[错误] GPS模块不可用，请检查 Gps/gps.py 和 Gps/gps_navigation_controller.py 是否存在")
                sys.exit(1)

            print("\n" + "=" * 60)
            print("  GPS导航模式初始化")
            print("=" * 60)

            # 设置目标点
            controller.set_gps_target(args.target_lat, args.target_lon)

            # 初始化GPSNavigationController
            if not controller._init_gps_navigation(gps_port=args.gps_port, gps_baudrate=args.gps_baudrate):
                print("[错误] GPSNavigationController初始化失败")
                sys.exit(1)

            # 使用GPSNavigationController初始化并校准
            if not controller._gps_navigation.initialize():
                print("[错误] GPS导航初始化失败")
                sys.exit(1)

            # 初始化电机驱动（GPS模式也需要）
            if not controller._init_motor_driver():
                print("[错误] 电机驱动初始化失败")
                controller._gps_navigation.stop()
                sys.exit(1)

            # 获取航向角信息
            state = controller._gps_navigation.get_state()

            # 先更新一次导航状态，确保数据最新
            controller._gps_navigation._update_navigation()
            state = controller._gps_navigation.get_state()

            controller._gps_bearing = state.bearing_angle
            controller._gps_distance = state.distance_to_target
            controller._gps_enabled = True

            # 显示确认界面
            controller.confirm_target_heading(state.bearing_angle, state.distance_to_target)

            if args.confirm:
                try:
                    response = input("\n是否使用此目标航向角启动控制? (Y/n): ").strip().lower()
                    if response == 'n':
                        print("[取消] 已取消启动")
                        controller._gps_navigation.stop()
                        sys.exit(0)
                except EOFError:
                    pass

            # 启动航向锁定控制（使用GPS计算的目标航向）
            controller._target_heading = state.target_heading
            controller._pid.reset()
            controller._is_running = True
            print(f"[启动] 航向角锁定控制已启动，目标航向: {controller._target_heading:.1f}°")
            print(f"[PID] Kp={controller._pid.kp}, Ki={controller._pid.ki}, Kd={controller._pid.kd}")

            controller.run_loop(duration=args.duration)
        else:
            # 普通模式（无GPS）
            if controller.start(calibrate=True):
                controller.run_loop(duration=args.duration)
            else:
                print("\n初始化失败，请检查:")
                print("  1. 罗盘是否正确连接到串口")
                print("  2. 电机驱动GPIO引脚是否配置正确")
