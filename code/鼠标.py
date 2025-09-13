import cv2
from robomaster import robot

# 初始化机器人
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")  # 直连模式
ep_chassis = ep_robot.chassis
ep_led = ep_robot.led

# 参数
MAX_SPEED = 100
moving = False
window_name = "Robot Mouse Control"
last_x, last_y = None, None

# 鼠标回调函数
def mouse_event(event, x, y, flags, param):
    global moving, last_x, last_y

    # 左键按下 -> 开始移动
    if event == cv2.EVENT_LBUTTONDOWN:
        moving = True
        last_x, last_y = x, y
        print("Start moving")

    # 左键抬起 -> 停止
    elif event == cv2.EVENT_LBUTTONUP:
        moving = False
        ep_chassis.drive_speed(x=0, y=0, z=0)
        ep_led.set_led(comp="all", r=0, g=0, b=0)
        print("Stop moving")

    # 鼠标移动 -> 控制机器人
    elif event == cv2.EVENT_MOUSEMOVE and moving:
        if last_x is None or last_y is None:
            last_x, last_y = x, y
            return

        dx = x - last_x
        dy = y - last_y
        last_x, last_y = x, y

        # 映射到速度
        vx = -dy /10
        wz = dx *2

        # 死区
        if abs(vx) < 0.05:
            vx = 0
        if abs(wz) < 0.05:
            wz = 0

        # 限幅
        vx = max(min(vx, 1.0), -1.0) * MAX_SPEED
        wz = max(min(wz, 1.0), -1.0) * MAX_SPEED

        # 发送速度
        ep_chassis.drive_speed(x=vx, y=0, z=wz)

        # LED 控制（保持常亮）
        if dx < -2:   # 左转
            ep_led.set_led(comp="bottom_left", r=255, g=255, b=0, effect="on")
        elif dx > 2:  # 右转
            ep_led.set_led(comp="bottom_right", r=255, g=255, b=0, effect="on")
        else:
            # 不转弯时关掉侧灯
            ep_led.set_led(comp="bottom_left", r=0, g=0, b=0)
            ep_led.set_led(comp="bottom_right", r=0, g=0, b=0)

        if dy > 2:   # 后退 / 减速
            ep_led.set_led(comp="bottom_back", r=255, g=0, b=0, effect="on")
        else:
            ep_led.set_led(comp="bottom_back", r=0, g=0, b=0)

def main():
    global window_name
    # 创建窗口
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_event)

    print("按 ESC 退出程序")
    while True:
        # 画一个纯黑背景窗口
        img = 255 * np.ones((400, 600, 3), dtype=np.uint8)
        cv2.putText(img, "Use mouse inside this window", (50, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

        cv2.imshow(window_name, img)

        key = cv2.waitKey(20) & 0xFF
        if key == 27:  # ESC 键退出
            break

    # 清理
    ep_chassis.drive_speed(x=0, y=0, z=0)
    ep_led.set_led(comp="all", r=0, g=0, b=0)
    ep_robot.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    import numpy as np
    main()
