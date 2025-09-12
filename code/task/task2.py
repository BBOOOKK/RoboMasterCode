from robomaster import robot
import time

class RobotLEDController:
    def __init__(self):
        # 初始化机器人
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize()
        # 获取底盘和 LED 控制对象
        self.chassis = self.ep_robot.chassis
        self.led = self.ep_robot.led

    def set_led_forward(self):
        # 前进时：前 LED 白，其他熄灭
        self.led.set_led("front", "white")
        self.led.set_led("left", "black")
        self.led.set_led("right", "black")
        self.led.set_led("back", "black")

    def set_led_backward(self):
        # 后退时：前后 LED 白，左右熄灭
        self.led.set_led("front", "white")
        self.led.set_led("back", "white")
        self.led.set_led("left", "black")
        self.led.set_led("right", "black")

    def set_led_left(self):
        # 左转时：前 LED 白，左 LED 黄闪烁，其他熄灭
        self.led.set_led("front", "white")
        # 模拟闪烁，实际 SDK 若有闪烁方法可替换
        for _ in range(3):
            self.led.set_led("left", "yellow")
            time.sleep(0.3)
            self.led.set_led("left", "black")
            time.sleep(0.3)
        self.led.set_led("right", "black")
        self.led.set_led("back", "black")

    def set_led_right(self):
        # 右转时：前 LED 白，右 LED 黄闪烁，其他熄灭
        self.led.set_led("front", "white")
        # 模拟闪烁
        for _ in range(3):
            self.led.set_led("right", "yellow")
            time.sleep(0.3)
            self.led.set_led("right", "black")
            time.sleep(0.3)
        self.led.set_led("left", "black")
        self.led.set_led("back", "black")

    def move_and_control_led(self):
        while True:
            cmd = input("请输入指令（如 fd 1.5、back 1.2、left 90、right 30，输入 'exit' 退出）：")
            if cmd == 'exit':
                break
            parts = cmd.split()
            if len(parts) != 2:
                print("指令格式错误，请重新输入！")
                continue
            action, value = parts[0], parts[1]
            try:
                value = float(value)
            except ValueError:
                print("数值格式错误，请输入数字！")
                continue

            if action == 'fd':
                self.set_led_forward()
                self.chassis.move(x=value, y=0, z=0)
            elif action == 'back':
                self.set_led_backward()
                self.chassis.move(x=-value, y=0, z=0)
            elif action == 'left':
                self.set_led_left()
                self.chassis.move(x=0, y=0, z=value)
            elif action == 'right':
                self.set_led_right()
                self.chassis.move(x=0, y=0, z=-value)
            else:
                print("无效指令，请重新输入！")

    def close(self):
        self.ep_robot.close()

if __name__ == "__main__":
    controller = RobotLEDController()
    try:
        controller.move_and_control_led()
    finally:
        controller.close()