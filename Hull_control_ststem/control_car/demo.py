import pigpio
import time
pi = pigpio.pi() #创建 pigpio 对象
LED_PIN = 35 #定义 LED 连接的 GPIO 口
#淘宝直销：店铺搜索
APISQUEEN
PWM_FREQUENCY = 50 #定义 PWM 频率，单位 Hz
PWM_range=1000
PWM_DUTYCYCLE = 0 #定义 PWM 占空比，取值范围 0⑵55、
pi.set_mode(LED_PIN, pigpio.OUTPUT) #设置 GPIO 口为输出模式
pi.set_PWM_frequency(LED_PIN, PWM_FREQUENCY) #设置 PWM 频率
pi.set_PWM_range(LED_PIN, PWM_range) # set range 1000
pi.set_PWM_dutycycle(LED_PIN, 75) #设置 PWM 占空比 75/1000=7.5%
time.sleep(3) #延迟 3s 解锁成功
# 3. 控制电调
# 占空比 75% 实际占空比 7.5%-1.5ms（1.475ms 到 1.525ms 之间）时停转；
# 占空比 50%-75% 实际占空比 5%-7.5% 1ms-1.5ms 反转；
# 占空比 100% 实际占空比 7.5%-10% 1.5 ms -2ms 正转
pi.set_PWM_dutycycle(LED_PIN, 100)
# 正转 7.5%-10%占空比越大，正转速度越快
time.sleep(15)
pi.set_PWM_dutycycle(LED_PIN, 60)
# 反转 占空比越接近 5%，反转速度越快
time.sleep(5)
pi.set_PWM_dutycycle(LED_PIN,75)
# 停转
time.sleep(5)