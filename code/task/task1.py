from robomaster import robot
import importlib.util
import os
import time
import threading
import termios
import tty
import sys

# 动态加载 base1.py
HERE = os.path.dirname(__file__)
def _load_module(path_name):
    path = os.path.join(HERE, path_name)
    spec = importlib.util.spec_from_file_location(path_name[:-3], path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

base1 = _load_module("../Chassis-movement/base1.py")

# 从 base1 加载函数
safe_move = getattr(base1, "safe_move", None)
_wait_action = getattr(base1, "_wait_action", None)

# 获取单个按键输入
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# 设置 LED 灯光模式
def set_led_pattern(ep_robot, front=None, left=None, right=None, back=None):
    led = getattr(ep_robot, "led", None)
    if not led:
        return
    if front:
        led.set_led("front", *front)
    if left:
        led.set_led("left", *left)
    if right:
        led.set_led("right", *right)
    if back:
        led.set_led("back", *back)

# 添加运动状态控制类，实现持续运动与即时停止
class MotionController:
    def __init__(self, chassis, ep_robot):
        self.chassis = chassis
        self.ep_robot = ep_robot
        self.current_motion = None
        self.motion_thread = None
        self.stop_event = threading.Event()
        self.motion_lock = threading.Lock()

    def start_motion(self, vx=0, vy=0, yaw=0):
        """开始持续运动"""
        with self.motion_lock:
            self.stop_current_motion()
            self.current_motion = (vx, vy, yaw)
            self.stop_event.clear()
            self.motion_thread = threading.Thread(
                target=self._continuous_motion,
                daemon=True
            )
            self.motion_thread.start()

    def stop_current_motion(self):
        """停止当前运动"""
        if self.motion_thread and self.motion_thread.is_alive():
            self.stop_event.set()
            self.motion_thread.join(timeout=0.1)
        self.current_motion = None
        try:
            safe_move(self.chassis, x=0, y=0, z=0, wait=True)
        except Exception:
            pass

    def _continuous_motion(self):
        """持续运动处理线程"""
        vx, vy, yaw = self.current_motion
        step_time = 0.05  # 更新频率
        
        # 设置相应 LED
        if vx > 0:
            set_led_pattern(self.ep_robot, front=(255,255,255), left=(0,0,0), right=(0,0,0), back=(0,0,0))
        elif vx < 0:
            set_led_pattern(self.ep_robot, front=(255,255,255), left=(0,0,0), right=(0,0,0), back=(255,255,255))
        elif yaw > 0:
            set_led_pattern(self.ep_robot, front=(255,255,255), left=(0,0,0), right=(255,170,0), back=(0,0,0))
        elif yaw < 0:
            set_led_pattern(self.ep_robot, front=(255,255,255), left=(255,170,0), right=(0,0,0), back=(0,0,0))
        else:
            set_led_pattern(self.ep_robot, front=(255,255,255), left=(0,0,0), right=(0,0,0), back=(0,0,0))
        
        try:
            while not self.stop_event.is_set():
                dx = vx * step_time
                dy = vy * step_time
                dz = yaw * step_time
                try:
                    # 等待当前动作完成后再发送下一步指令
                    safe_move(self.chassis, x=dx, y=dy, z=dz, wait=True)
                except Exception as e:
                    # 捕捉并忽略连续运动中可能出现的异常
                    pass
                time.sleep(step_time)
        finally:
            set_led_pattern(self.ep_robot, front=(0,0,0), left=(0,0,0), right=(0,0,0), back=(0,0,0))

# 主控制循环，采用按键按下持续运动，松开立即停止
def control_loop(ep_robot, chassis, linear_speed, angular_speed):
    print("控制说明：")
    print("  按下 W/S: 前进/后退")
    print("  按下 A/D: 左移/右移")
    print("  按下 Q/E: 左转/右转")
    print("  松开控制键: 即刻停止运动")
    print("  空格: 急停")
    print("  X: 退出控制")
    print("请开始操作...")

    controller = MotionController(chassis, ep_robot)
    running = True

    # 状态保存当前按下的键
    pressed_key = None

    while running:
        key = getch().lower()

        # 处理急停与退出
        if key == " ":
            controller.stop_current_motion()
            pressed_key = None
            continue
        if key == "x":
            print("退出控制...")
            controller.stop_current_motion()
            running = False
            continue

        # 对每个运动命令，按下时启动运动，松开时停止
        if key in ("w", "s", "a", "d", "q", "e"):
            # 如果当前没有按下或与前一按键不同，则启动新的运动
            if pressed_key != key:
                pressed_key = key
                if key == "w":
                    controller.start_motion(vx=linear_speed)
                elif key == "s":
                    controller.start_motion(vx=-linear_speed)
                elif key == "a":
                    controller.start_motion(vy=-linear_speed)
                elif key == "d":
                    controller.start_motion(vy=linear_speed)
                elif key == "q":
                    controller.start_motion(yaw=-angular_speed)
                elif key == "e":
                    controller.start_motion(yaw=angular_speed)
        else:
            # 非运动键松开后停止运动
            controller.stop_current_motion()
            pressed_key = None

# 主函数
def main():
    ep_robot = None
    try:
        ep_robot = robot.Robot()
        ep_robot.initialize()
        chassis = ep_robot.chassis
        print("机器人已连接，进入控制模式。")

        # 用户设置速度
        linear_speed = float(input("请输入线速度 (m/s) [默认 1.0]: ") or "1.0")
        angular_speed = float(input("请输入角速度 (deg/s) [默认 120]: ") or "120")

        control_loop(ep_robot, chassis, linear_speed, angular_speed)
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if ep_robot:
            ep_robot.close()
        print("已断开机器人连接。")

if __name__ == "__main__":
    main()