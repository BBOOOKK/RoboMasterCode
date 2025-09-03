from robomaster import robot
import cv2
import time

if __name__ == '__main__':
    # 连接机器人
    ep_robot = robot.Robot(ip="192.168.10.2")
    ep_robot.initialize()
    
    # 初始化摄像头（默认开启RGB摄像头，分辨率640x480）
    camera = ep_robot.camera
    camera.start_video_stream(display=False)  # 不开启机器人自带显示屏
    print("摄像头启动，按 'q' 退出...")
    
    try:
        while True:
            # 获取一帧图像（BGR格式，适合OpenCV处理）
            frame = camera.read_cv2_image()
            if frame is None:
                continue
            
            # 在窗口显示图像
            cv2.imshow("RoboMaster Camera", frame)
            
            # 按 's' 保存当前帧
            key = cv2.waitKey(1)
            if key == ord('s'):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f"robot_capture_{timestamp}.jpg", frame)
                print(f"图像已保存：robot_capture_{timestamp}.jpg")
            # 按 'q' 退出
            elif key == ord('q'):
                break
    finally:
        # 释放资源
        cv2.destroyAllWindows()
        camera.stop_video_stream()
        ep_robot.close()
        print("视觉采集结束")