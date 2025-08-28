import cv2
import robomaster
from robomaster import robot
import time

def main():
    # 配置本地IP（如果自动获取失败，手动指定）
    # robomaster.config.LOCAL_IP_STR = "192.168.2.20"  # 替换为你的电脑IP

    # 初始化机器人（AP直连模式）
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='ap')

    try:
        # 验证机器人连接
        version = ep_robot.get_version()
        print(f"Robot version: {version}")

        # 初始化摄像头模块（旧版SDK接口）
        ep_camera = ep_robot.camera
        ep_camera.start_video_stream(display=False)  # 不显示默认窗口，用OpenCV显示
        print("摄像头启动成功，按 'q' 退出")

        # 循环获取并处理图像
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

            # 按 'q' 键退出循环
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"视觉处理错误: {str(e)}")

    finally:
        # 释放资源
        cv2.destroyAllWindows()  # 关闭所有OpenCV窗口
        ep_camera.stop_video_stream()  # 停止摄像头流
        ep_robot.close()  # 断开机器人连接
        print("程序已退出，资源已释放")

if __name__ == '__main__':
    main()

