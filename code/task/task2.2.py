from robomaster import robot
import time

class LEDTester:
    def __init__(self):
        # 初始化机器人连接
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize()
        # 获取LED控制对象
        self.led = self.ep_robot.led

    def set_led_forward(self):
        """设置前进时的LED灯效：前LED白，其他熄灭"""
        self.led.set_led("front", "white")
        self.led.set_led("left", "black")
        self.led.set_led("right", "black")
        self.led.set_led("back", "black")
        print("已设置前进LED灯效")

    def set_led_backward(self):
        """设置后退时的LED灯效：前后LED白，左右熄灭"""
        self.led.set_led("front", "white")
        self.led.set_led("back", "white")
        self.led.set_led("left", "black")
        self.led.set_led("right", "black")
        print("已设置后退LED灯效")

    def set_led_left(self):
        """设置左转时的LED灯效：前LED白，左LED黄闪烁，其他熄灭"""
        self.led.set_led("front", "white")
        # 模拟左LED闪烁
        for _ in range(3):
            self.led.set_led("left", "yellow")
            time.sleep(0.3)
            self.led.set_led("left", "black")
            time.sleep(0.3)
        self.led.set_led("right", "black")
        self.led.set_led("back", "black")
        print("已设置左转LED灯效（左LED闪烁）")

    def set_led_right(self):
        """设置右转时的LED灯效：前LED白，右LED黄闪烁，其他熄灭"""
        self.led.set_led("front", "white")
        # 模拟右LED闪烁
        for _ in range(3):
            self.led.set_led("right", "yellow")
            time.sleep(0.3)
            self.led.set_led("right", "black")
            time.sleep(0.3)
        self.led.set_led("left", "black")
        self.led.set_led("back", "black")
        print("已设置右转LED灯效（右LED闪烁）")

    def test_all_led_effects(self):
        """依次测试所有LED灯效"""
        print("开始LED测试...")
        self.set_led_forward()
        time.sleep(2)  # 展示效果2秒
        self.set_led_backward()
        time.sleep(2)
        self.set_led_left()
        time.sleep(2)
        self.set_led_right()
        time.sleep(2)
        # 测试结束后熄灭所有LED
        self.led.set_led("front", "black")
        self.led.set_led("back", "black")
        self.led.set_led("left", "black")
        self.led.set_led("right", "black")
        print("LED测试结束，所有LED已熄灭")

    def close(self):
        """关闭机器人连接"""
        self.ep_robot.close()

if __name__ == "__main__":
    tester = LEDTester()
    try:
        tester.test_all_led_effects()
    finally:
        tester.close()