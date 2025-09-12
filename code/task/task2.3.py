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
        # 颜色的 RGB 值
        self.WHITE = (255, 255, 255)
        self.YELLOW = (255, 255, 0)
        self.BLACK = (0, 0, 0)

    def set_led_forward(self):
        # 前进时：前 LED 白，其他熄灭
        self.led.set_led(comp="bottom_front", r=self.WHITE[0], g=self.WHITE[1], b=self.WHITE[2], effect="on")
        self.led.set_led(comp="bottom_left", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")
        self.led.set_led(comp="bottom_right", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")
        self.led.set_led(comp="bottom_back", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")

    def set_led_backward(self):
        # 后退时：前后 LED 白，左右熄灭
        self.led.set_led(comp="bottom_front", r=self.WHITE[0], g=self.WHITE[1], b=self.WHITE[2], effect="on")
        self.led.set_led(comp="bottom_back", r=self.WHITE[0], g=self.WHITE[1], b=self.WHITE[2], effect="on")
        self.led.set_led(comp="bottom_left", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")
        self.led.set_led(comp="bottom_right", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")

    def set_led_left(self):
        # 左转时：前 LED 白，左 LED 黄闪烁，其他熄灭
        self.led.set_led(comp="bottom_front", r=self.WHITE[0], g=self.WHITE[1], b=self.WHITE[2], effect="on")
        # 左 LED 闪烁
        self.led.set_led(comp="bottom_left", r=self.YELLOW[0], g=self.YELLOW[1], b=self.YELLOW[2], effect="flash", freq=2)
        self.led.set_led(comp="bottom_right", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")
        self.led.set_led(comp="bottom_back", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")

    def set_led_right(self):
        # 右转时：前 LED 白，右 LED 黄闪烁，其他熄灭
        self.led.set_led(comp="bottom_front", r=self.WHITE[0], g=self.WHITE[1], b=self.WHITE[2], effect="on")
        # 右 LED 闪烁
        self.led.set_led(comp="bottom_right", r=self.YELLOW[0], g=self.YELLOW[1], b=self.YELLOW[2], effect="flash", freq=2)
        self.led.set_led(comp="bottom_left", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")
        self.led.set_led(comp="bottom_back", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")

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
        # 关闭前熄灭所有LED
        self.led.set_led(comp="bottom_front", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")
        self.led.set_led(comp="bottom_back", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")
        self.led.set_led(comp="bottom_left", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")
        self.led.set_led(comp="bottom_right", r=self.BLACK[0], g=self.BLACK[1], b=self.BLACK[2], effect="off")
        self.ep_robot.close()

if __name__ == "__main__":
    controller = RobotLEDController()
    try:
        controller.move_and_control_led()
    finally:
        controller.close()
        