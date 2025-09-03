from robomaster import robot
import cv2
import numpy as np

def detect_color(frame, lower_bound, upper_bound, color):
    """识别指定颜色的物体，返回标记后的图像和目标中心坐标"""
    # 转换到HSV色彩空间（更适合颜色识别）
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 根据阈值创建掩码
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    # 去除噪点
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    # 查找轮廓
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center = None
    # 绘制最大轮廓
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        # 计算轮廓矩，获取中心坐标
        M = cv2.moments(c)
        if M["m00"] > 0:
            center = (int(M["cx"]), int(M["cy"]))
        # 只标记较大的目标（过滤噪声）
        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), color, 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)  # 中心点
    return frame, center

if __name__ == '__main__':
    ep_robot = robot.Robot(ip="192.168.10.2")
    ep_robot.initialize()
    camera = ep_robot.camera
    camera.start_video_stream(display=False)
    
    # 定义颜色阈值（HSV范围，这里以红色为例）
    red_lower = np.array([0, 120, 70])    # 低阈值
    red_upper = np.array([10, 255, 255])  # 高阈值（红色在HSV中分为两段，这里取一段）
    red_color = (0, 0, 255)  # BGR格式的红色（OpenCV中颜色通道为BGR）
    
    print("开始红色目标识别，按 'q' 退出...")
    try:
        while True:
            frame = camera.read_cv2_image()
            if frame is None:
                continue
            # 识别红色目标
            marked_frame, center = detect_color(frame, red_lower, red_upper, red_color)
            # 显示目标中心坐标
            if center:
                cv2.putText(marked_frame, f"Center: {center}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Color Detection", marked_frame)
            if cv2.waitKey(1) == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()
        camera.stop_video_stream()
        ep_robot.close()