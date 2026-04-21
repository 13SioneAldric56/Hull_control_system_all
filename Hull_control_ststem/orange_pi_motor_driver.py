#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
香橙派5电机驱动控制示例
基于STM32 PWM驱动模式的香橙派实现

硬件连接说明：
- PWM输出：香橙派5的PWM引脚 (需配置为PWM模式)
- 编码器输入：香橙派5的GPIO引脚 (用于读取编码器A/B相)
- 控制引脚：使能、方向、刹车等GPIO控制
"""

import RPi.GPIO as GPIO
import PWM  # 香橙派PWM库 (需要安装)
import time
import threading
from enum import Enum
from dataclasses import dataclass
from typing import Optional
import yaml  # 需要安装PyYAML


# 加载配置
def load_config(config_path: str = "motor_config.yaml"):
    """加载电机配置文件"""
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    return config


class ControlMode(Enum):
    """控制模式枚举"""
    VOLTAGE = "voltage"    # 电压模式
    TORQUE = "torque"      # 转矩模式 (电流环)
    SPEED = "speed"        # 速度模式
    POSITION = "position"  # 位置模式


@dataclass
class MotorState:
    """电机状态数据结构"""
    speed: float = 0.0          # 转速 (RPM)
    position: float = 0.0       # 位置 (脉冲数)
    current: float = 0.0        # 电流 (A)
    voltage: float = 0.0        # 电压 (V)
    temperature: float = 25.0   # 温度 (°C)
    fault_flag: bool = False    # 故障标志


class MotorController:
    """电机控制器类"""

    def __init__(self, config_path: str = "motor_config.yaml"):
        # 加载配置
        self.config = load_config(config_path)
        self.pwm_cfg = self.config['pwm']
        self.encoder_cfg = self.config['encoder']
        self.motor_cfg = self.config['motor']
        self.gpio_cfg = self.config['gpio']
        self.control_cfg = self.config['control_mode']
        self.protect_cfg = self.config['protection']

        # 初始化状态
        self.state = MotorState()
        self.target_speed = 0.0
        self.target_torque = 0.0
        self.target_position = 0.0
        self.control_mode = ControlMode(self.control_cfg['mode'])

        # 编码器计数器
        self.encoder_count = 0
        self.last_encoder_count = 0
        self.encoder_lock = threading.Lock()

        # 初始化硬件
        self._init_gpio()
        self._init_pwm()
        self._init_encoder()

        # 启动控制线程
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

        # 启动编码器读取线程
        self.encoder_thread = threading.Thread(target=self._encoder_loop, daemon=True)
        self.encoder_thread.start()

    def _init_gpio(self):
        """初始化GPIO"""
        # 设置GPIO模式 (香橙派5使用BCM编号)
        GPIO.setmode(GPIO.BCM)

        # 设置使能引脚为输出
        enable_pin = self._parse_gpio_pin(self.gpio_cfg['enable_pin'])
        GPIO.setup(enable_pin, GPIO.OUT)
        GPIO.output(enable_pin, GPIO.LOW)  # 默认禁用

        # 设置方向引脚 (如有)
        if 'direction_pin' in self.gpio_cfg:
            dir_pin = self._parse_gpio_pin(self.gpio_cfg['direction_pin'])
            GPIO.setup(dir_pin, GPIO.OUT)
            GPIO.output(dir_pin, GPIO.LOW)

        # 设置刹车引脚 (如有)
        if 'brake_pin' in self.gpio_cfg:
            brake_pin = self._parse_gpio_pin(self.gpio_cfg['brake_pin'])
            GPIO.setup(brake_pin, GPIO.OUT)
            GPIO.output(brake_pin, GPIO.HIGH)  # 默认刹车释放

        # 设置故障检测引脚为输入
        if 'fault_pin' in self.gpio_cfg:
            fault_pin = self._parse_gpio_pin(self.gpio_cfg['fault_pin'])
            GPIO.setup(fault_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def _init_pwm(self):
        """初始化PWM输出"""
        self.pwm_channels = []
        pwm_pins = self.gpio_cfg['pwm_pin']

        for i, pin in enumerate(pwm_pins):
            # 创建PWM通道 (需要根据香橙派5的实际PWM库调整)
            # 假设有PWM库，频率和分辨率可配置
            pwm = PWM.PWM(channel=i, frequency=self.pwm_cfg['frequency'])
            pwm.start(0)  # 初始占空比0%
            self.pwm_channels.append({
                'pwm': pwm,
                'pin': pin,
                'duty': 0.0
            })

    def _init_encoder(self):
        """初始化编码器接口"""
        encoder_pin_a = self._parse_gpio_pin(self.gpio_cfg['encoder_pin']['a'])
        encoder_pin_b = self._parse_gpio_pin(self.gpio_cfg['encoder_pin']['b'])

        # 设置编码器引脚为输入，带上拉电阻
        GPIO.setup(encoder_pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(encoder_pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # 添加中断回调 (根据香橙派GPIO库调整)
        # 注意：并非所有GPIO都支持中断，需要根据实际情况选择引脚
        try:
            GPIO.add_event_detect(encoder_pin_a, GPIO.BOTH,
                                  callback=self._encoder_callback)
            GPIO.add_event_detect(encoder_pin_b, GPIO.BOTH,
                                  callback=self._encoder_callback)
        except RuntimeError:
            print("警告：编码器引脚不支持中断，将使用轮询方式读取")

    def _parse_gpio_pin(self, pin_str: str) -> int:
        """解析GPIO引脚编号 (如 'PA0' -> GPIO编号)"""
        # 香橙派5的引脚编号方式可能不同，需要根据实际板卡调整
        # 这里假设使用BCM编号，需要映射表
        pin_map = {
            'PA0': 0,   # 根据实际硬件修改
            'PA1': 1,
            'PA2': 2,
            'PB0': 8,
            'PB1': 9,
            'PC13': 13,
            'PC14': 14,
            'PC15': 15,
            'PA15': 15,
        }
        return pin_map.get(pin_str, int(pin_str) if pin_str.isdigit() else 0)

    def _encoder_callback(self, channel):
        """编码器中断回调函数"""
        with self.encoder_lock:
            pin_a = GPIO.input(self._parse_gpio_pin(self.gpio_cfg['encoder_pin']['a']))
            pin_b = GPIO.input(self._parse_gpio_pin(self.gpio_cfg['encoder_pin']['b']))

            # 简单的正交解码
            # 根据A相和B相的状态决定计数方向
            # 实际实现需要根据具体编码器输出逻辑调整
            if pin_a and not pin_b:
                self.encoder_count += 1
            elif not pin_a and pin_b:
                self.encoder_count -= 1
            elif pin_a and pin_b:
                # 可能是一个完整脉冲的结束，需要状态机处理
                pass

    def _encoder_loop(self):
        """编码器读取线程 (轮询方式)"""
        last_a = 0
        last_b = 0

        while self.running:
            pin_a = GPIO.input(self._parse_gpio_pin(self.gpio_cfg['encoder_pin']['a']))
            pin_b = GPIO.input(self._parse_gpio_pin(self.gpio_cfg['encoder_pin']['b']))

            # 状态变化检测
            if pin_a != last_a or pin_b != last_b:
                with self.encoder_lock:
                    # 正交解码逻辑
                    if last_a == 0 and last_b == 0:
                        if pin_a == 1 and pin_b == 0:
                            self.encoder_count += 1
                        elif pin_a == 0 and pin_b == 1:
                            self.encoder_count -= 1
                    elif last_a == 1 and last_b == 0:
                        if pin_a == 1 and pin_b == 1:
                            self.encoder_count += 1
                        elif pin_a == 0 and pin_b == 0:
                            self.encoder_count -= 1
                    # ... 完整的状态机需要4个状态

                last_a = pin_a
                last_b = pin_b

            time.sleep(0.00001)  # 10μs轮询间隔，可检测高速脉冲

    def _control_loop(self):
        """控制主循环"""
        dt = 0.001  # 控制周期1ms

        while self.running:
            try:
                # 读取编码器位置
                with self.encoder_lock:
                    current_count = self.encoder_count

                # 计算速度 (RPM)
                delta_count = current_count - self.last_encoder_count
                self.last_encoder_count = current_count

                # 转换为速度: (脉冲数/周期) * 60秒 / 编码器分辨率
                speed_rpm = (delta_count / dt) * 60 / self.encoder_cfg['resolution']
                self.state.speed = speed_rpm
                self.state.position = current_count

                # 根据控制模式执行控制算法
                if self.control_mode == ControlMode.SPEED:
                    self._speed_control(dt)
                elif self.control_mode == ControlMode.TORQUE:
                    self._torque_control(dt)
                elif self.control_mode == ControlMode.POSITION:
                    self._position_control(dt)
                elif self.control_mode == ControlMode.VOLTAGE:
                    self._voltage_control()

                # 检查保护
                self._check_protection()

                time.sleep(dt)

            except Exception as e:
                print(f"控制循环错误: {e}")

    def _speed_control(self, dt: float):
        """速度环控制"""
        # 获取PID参数
        pid = self.control_cfg['speed_pid']
        error = self.target_speed - self.state.speed

        # PID计算 (简化版，实际需要积分分离、抗积分饱和等)
        p_term = pid['kp'] * error

        # 积��项 (简单实现)
        if not hasattr(self, 'speed_integral'):
            self.speed_integral = 0.0
        self.speed_integral += error * dt
        i_term = pid['ki'] * self.speed_integral

        # 输出限幅
        output = max(-pid['limit'], min(pid['limit'], p_term + i_term))

        # 转换为PWM占空比
        # 假设电机额定电压对应100%占空比
        max_voltage = 24.0  # 假设24V系统
        duty_cycle = (output / max_voltage) * 100
        duty_cycle = max(-100, min(100, duty_cycle))

        self._set_pwm_duty(duty_cycle)

    def _torque_control(self, dt: float):
        """转矩控制 (电流环) - FOC模式"""
        # 这里需要电流反馈，假设有电流传感器
        # 简化实现：直接设置电流给定值
        target_current = self.target_torque  # 转矩与电流成正比

        # 电流环PID
        pid = self.control_cfg['current_pid']['q_pid']
        error = target_current - self.state.current

        p_term = pid['kp'] * error
        if not hasattr(self, 'current_integral'):
            self.current_integral = 0.0
        self.current_integral += error * dt
        i_term = pid['ki'] * self.current_integral

        output = max(-pid['limit'], min(pid['limit'], p_term + i_term))

        # 转换为PWM占空比
        max_voltage = 24.0
        duty_cycle = (output / max_voltage) * 100
        duty_cycle = max(-100, min(100, duty_cycle))

        self._set_pwm_duty(duty_cycle)

    def _position_control(self, dt: float):
        """位置环控制"""
        # 位置环 -> 速度环 -> 电流环的级联控制
        error = self.target_position - self.state.position

        # 位置PID (简单比例控制)
        kp_pos = 0.1
        self.target_speed = kp_pos * error

        # 限制目标速度
        max_speed = self.motor_cfg['max_rpm']
        self.target_speed = max(-max_speed, min(max_speed, self.target_speed))

        # 调用速度环
        self._speed_control(dt)

    def _voltage_control(self):
        """直接电压控制"""
        # 直接设置PWM占空比，无闭环
        max_voltage = 24.0
        duty_cycle = (self.target_speed / max_voltage) * 100  # 复用target_speed作为电压给定
        duty_cycle = max(-100, min(100, duty_cycle))
        self._set_pwm_duty(duty_cycle)

    def _set_pwm_duty(self, duty_cycle: float):
        """设置PWM占空比"""
        # 假设单路PWM，双向控制需要H桥电路
        # 正转：PWM通道输出占空比
        # 反转：可能需要另一路PWM或方向引脚

        if len(self.pwm_channels) >= 1:
            pwm_ch = self.pwm_channels[0]
            # 设置占空比 (0-100%)
            pwm_ch['pwm'].setDutyCycle(abs(duty_cycle))

            # 如有方向控制引脚
            if 'direction_pin' in self.gpio_cfg:
                dir_pin = self._parse_gpio_pin(self.gpio_cfg['direction_pin'])
                GPIO.output(dir_pin, GPIO.HIGH if duty_cycle >= 0 else GPIO.LOW)

    def _check_protection(self):
        """检查保护条件"""
        # 读取故障引脚
        if 'fault_pin' in self.gpio_cfg:
            fault_pin = self._parse_gpio_pin(self.gpio_cfg['fault_pin'])
            if GPIO.input(fault_pin) == GPIO.LOW:  # 低电平表示故障
                self.state.fault_flag = True
                self._emergency_stop()
                return

        # 过流保护 (需电流采样)
        # if abs(self.state.current) > self.protect_cfg['overcurrent_threshold']:
        #     self.state.fault_flag = True
        #     self._emergency_stop()

        # 欠压/过压保护 (需电压采样)
        # if self.state.voltage < self.protect_cfg['undervoltage_threshold'] or \
        #    self.state.voltage > self.protect_cfg['overvoltage_threshold']:
        #     self.state.fault_flag = True
        #     self._emergency_stop()

    def _emergency_stop(self):
        """紧急停止"""
        print("故障！紧急停止")
        self._set_pwm_duty(0)
        if 'brake_pin' in self.gpio_cfg:
            brake_pin = self._parse_gpio_pin(self.gpio_cfg['brake_pin'])
            GPIO.output(brake_pin, GPIO.LOW)  # 刹车有效

    def enable(self):
        """使能电机"""
        enable_pin = self._parse_gpio_pin(self.gpio_cfg['enable_pin'])
        GPIO.output(enable_pin, GPIO.HIGH)
        print("电机已使能")

    def disable(self):
        """禁用电机"""
        enable_pin = self._parse_gpio_pin(self.gpio_cfg['enable_pin'])
        GPIO.output(enable_pin, GPIO.LOW)
        self._set_pwm_duty(0)
        print("电机已禁用")

    def set_speed(self, speed_rpm: float):
        """设置目标速度 (RPM)"""
        max_speed = self.motor_cfg['max_rpm']
        self.target_speed = max(-max_speed, min(max_speed, speed_rpm))
        print(f"设置速度: {self.target_speed:.1f} RPM")

    def set_torque(self, torque_current: float):
        """设置目标转矩 (电流A)"""
        max_current = self.motor_cfg['rated_current']
        self.target_torque = max(-max_current, min(max_current, torque_current))
        print(f"设置转矩电流: {self.target_torque:.2f} A")

    def set_position(self, position: float):
        """设置目标位置 (脉冲数)"""
        self.target_position = position
        print(f"设置位置: {self.target_position:.0f} 脉冲")

    def get_state(self) -> MotorState:
        """获取电机状态"""
        return self.state

    def stop(self):
        """停止电机"""
        self.target_speed = 0
        self.target_torque = 0
        self._set_pwm_duty(0)
        print("电机停止")

    def cleanup(self):
        """清理资源"""
        self.running = False
        self.control_thread.join(timeout=1.0)
        self.encoder_thread.join(timeout=1.0)

        # 停止PWM
        for ch in self.pwm_channels:
            ch['pwm'].stop()

        # 清理GPIO
        GPIO.cleanup()
        print("资源已清理")


# ============ 使用示例 ============
def main():
    """主函数示例"""
    print("=== 香橙派5电机控制器示例 ===\n")

    # 创建控制器
    controller = MotorController("motor_config.yaml")

    try:
        # 使能电机
        controller.enable()
        time.sleep(0.5)

        # 设置速度模式
        controller.control_mode = ControlMode.SPEED

        # 加速运行
        print("\n--- 速度控制测试 ---")
        controller.set_speed(1000)  # 1000 RPM
        time.sleep(3)

        # 读取状态
        state = controller.get_state()
        print(f"当前速度: {state.speed:.1f} RPM")
        print(f"当前位置: {state.position:.0f} 脉冲")

        # 减速
        controller.set_speed(500)
        time.sleep(2)

        # 反转
        controller.set_speed(-800)
        time.sleep(2)

        # 停止
        controller.stop()
        time.sleep(1)

        # 位置控制测试
        print("\n--- 位置控制测试 ---")
        controller.control_mode = ControlMode.POSITION
        controller.set_position(10000)  # 10000脉冲
        time.sleep(3)

        state = controller.get_state()
        print(f"当前位置: {state.position:.0f} 脉冲")

        # 返回到原点
        controller.set_position(0)
        time.sleep(3)

    except KeyboardInterrupt:
        print("\n用户中断")

    finally:
        # 清理
        controller.disable()
        controller.cleanup()


if __name__ == "__main__":
    main()
