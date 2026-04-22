import time

# ===================== 官方表格引脚定义（完全对应你的截图） =====================
# PWM 调速引脚：GPIO1_A3  编号35  功能 PWM1_M2
PWM_GPIO_NUM = 35
PWM_CHIP = "pwmchip0"   # PWM1_M2 官方对应内核节点
PWM_DEV = "pwm0"
PERIOD_NS = 50000       # 电机驱动标准 20kHz 周期

# H桥方向控制引脚
IN1 = 52    # GPIO1_C4  方向引脚1
IN2 = 92    # GPIO2_D4  方向引脚2

# ===================== GPIO 通用控制函数 =====================
def gpio_export(pin):
    """导出GPIO引脚"""
    try:
        with open("/sys/class/gpio/export", "w") as f:
            f.write(f"{pin}\n")
        time.sleep(0.1)
    except OSError:
        # 引脚已导出则跳过报错
        pass

def gpio_set_output(pin):
    """设置引脚为输出模式"""
    with open(f"/sys/class/gpio/gpio{pin}/direction", "w") as f:
        f.write("out")

def gpio_write(pin, level):
    """输出电平 1=高  0=低"""
    with open(f"/sys/class/gpio/gpio{pin}/value", "w") as f:
        f.write(f"{level}")

# ===================== 硬件PWM 控制函数 =====================
def pwm_init():
    """初始化PWM通道"""
    try:
        with open(f"/sys/class/pwm/{PWM_CHIP}/export", "w") as f:
            f.write("0\n")
        time.sleep(0.2)
    except OSError:
        pass
    # 设置PWM周期 20kHz
    with open(f"/sys/class/pwm/{PWM_CHIP}/{PWM_DEV}/period", "w") as f:
        f.write(f"{PERIOD_NS}")

def pwm_set_duty(duty_percent):
    """
    设置PWM占空比 范围 0~100 %
    """
    duty_percent = max(0, min(100, duty_percent))
    duty_ns = int(PERIOD_NS * duty_percent / 100)
    with open(f"/sys/class/pwm/{PWM_CHIP}/{PWM_DEV}/duty_cycle", "w") as f:
        f.write(f"{duty_ns}")

def pwm_enable():
    """开启PWM波形输出"""
    with open(f"/sys/class/pwm/{PWM_CHIP}/{PWM_DEV}/enable", "w") as f:
        f.write("1")

def pwm_disable():
    """关闭PWM输出"""
    with open(f"/sys/class/pwm/{PWM_CHIP}/{PWM_DEV}/enable", "w") as f:
        f.write("0")

# ===================== H桥标准电机控制函数 =====================
def motor_forward(duty=50):
    """电机正转"""
    gpio_write(IN1, 1)
    gpio_write(IN2, 0)
    pwm_set_duty(duty)
    print(f"【正转】速度占空比：{duty}%")

def motor_backward(duty=50):
    """电机反转"""
    gpio_write(IN1, 0)
    gpio_write(IN2, 1)
    pwm_set_duty(duty)
    print(f"【反转】速度占空比：{duty}%")

def motor_stop_coast():
    """电机滑行停止（双低电平，惯性缓停）"""
    gpio_write(IN1, 0)
    gpio_write(IN2, 0)
    print("【停止】滑行缓停")

def motor_stop_brake():
    """电机主动刹车（双高电平，立刻刹停）"""
    gpio_write(IN1, 1)
    gpio_write(IN2, 1)
    print("【刹车】主动急停")

# ===================== 程序初始化 =====================
print("===== 引脚初始化开始 =====")
# 初始化两个方向GPIO引脚
gpio_export(IN1)
gpio_export(IN2)
gpio_set_output(IN1)
gpio_set_output(IN2)

# 初始化PWM引脚
pwm_init()
pwm_enable()
print("===== 全部初始化完成 =====")

# ===================== 自动完整测试流程 =====================
if __name__ == "__main__":
    try:
        # 测试1：正转 30%低速
        motor_forward(30)
        time.sleep(2)

        # 测试2：正转 80%高速
        motor_forward(80)
        time.sleep(2)

        # 测试3：滑行停止
        motor_stop_coast()
        time.sleep(1)

        # 测试4：反转 40%中速
        motor_backward(40)
        time.sleep(2)

        # 测试5：反转 90%高速
        motor_backward(90)
        time.sleep(2)

        # 测试6：主动刹车
        motor_stop_brake()
        time.sleep(1)

        # 测试7：平滑渐变调速（0%~100%缓慢升速，再100%~0%缓慢降速）
        print("\n===== 无级渐变调速测试 =====")
        motor_forward(0)
        time.sleep(1)
        # 升速
        for duty in range(0, 101, 5):
            pwm_set_duty(duty)
            print(f"当前速度：{duty}%")
            time.sleep(0.15)
        # 降速
        for duty in range(100, -1, -5):
            pwm_set_duty(duty)
            print(f"当前速度：{duty}%")
            time.sleep(0.15)

        motor_stop_coast()
        print("\n✅ 全部测试流程完成！")

    except KeyboardInterrupt:
        # Ctrl+C 强制退出安全保护
        motor_stop_coast()
        pwm_disable()
        print("\n🛑 程序手动退出，电机已安全停止")