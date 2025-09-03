import time
import cv2
from robomaster import robot
from robomaster import led


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type = "ap")

    ep_led = ep_robot.led
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)  # 不显示默认窗口，用OpenCV显示
    print("摄像头启动成功，按 'q' 退出")
    print("who are u ?")

    while True:
            # 获取OpenCV格式的图像（BGR格式）
            img = ep_camera.read_cv2_image()
            if img is None:
                time.sleep(0.1)
                continue

            # === 基础视觉处理示例 ===
            # 1. 转换为灰度图
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # 2. 边缘检测
            edges = cv2.Canny(gray, 100, 200)

            # 3. 显示原图和处理后的图像
            cv2.imshow("Original Camera", img)
            cv2.imshow("Edge Detection", edges)

            while 1:
                name = input ("Please input: ")
                print("who are u ?")
                if name == "book" :
                    print("HELLO!")
                    for count in range(5):
                        ep_led.set_led(comp = led.COMP_ALL, r=0, g=255, b=0, effect = led.EFFECT_ON)
                        time.sleep(0.2)
                        ep_led.set_led(comp = led.COMP_ALL, r=0, g=255, b=0, effect = led.EFFECT_OFF)
                        time.sleep(0.2)
                else : 
                    print("?")
                    for count in range(5):
                        ep_led.set_led(comp = led.COMP_ALL, r=255, g=0, b=0, effect = led.EFFECT_ON)
                        time.sleep(0.2)
                        ep_led.set_led(comp = led.COMP_ALL, r=255, g=0, b=0, effect = led.EFFECT_OFF)
                        time.sleep(0.2)

            # 按 'q' 键退出循环
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break



    

    ep_robot.close()

