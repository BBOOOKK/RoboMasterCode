from robomaster import robot
import cv2
import numpy as np

def track_color_and_move(frame, center, chassis, screen_center=(320, 240)):
    """根据目标中心与画面中心的偏差，控制底盘移动追踪目标"""
    if not center:
        # 未识别到目标，停止移动
        chassis.stop()
        return "未识别到目标，停止移动"
    
    # 计算偏差（x偏差：左右方向，y偏差：前后方向）
    dx = center[0] - screen_center[0]  # 横向偏差（正值偏右，负值偏左）
    dy = screen_center[1] - center[1]  # 纵向偏差（正值目标偏上，可能距离较远）
    
    # 左右调整（偏差超过阈值时旋转）
    if abs(dx) > 30:
        rotate_speed = 0.3 if dx > 0 else -0.3  # 右偏则顺时针转，左偏则逆时针转
        chassis.drive_speed(x=0, y=0, z=rotate_speed)
    else:
        # 前后调整（目标大小通过y偏差粗略判断，实际可结合面积计算）
        if abs(dy) > 50:
            forward_speed = 0.2 if dy > 0 else -0.2  # 目标偏上（小）则前进，偏下（大）则后退
            chassis.drive_speed(x=forward_speed, y=0, z=0)
        else:
            chassis.stop()
            return "目标已对准，停止移动"
    
    return f"追踪中：横向偏差 {dx}，纵向偏差 {dy}"

if __name__ == '__main__':
    ep_robot = robot.Robot(ip="192.168.10.2")
    ep_robot.initialize()
    camera = ep_robot.camera
    camera.start_video_stream(display=False)
    chassis = ep_robot.chassis
    
    # 红色目标阈值（同二级）
    red_lower = np.array([0, 120, 70])
    red_upper = np.array([10, 255, 255])
    screen_center = (320, 240)  # 640x480分辨率的画面中心
    
    print("开始目标追踪，按 'q' 退出...")
    try:
        while True:
            frame = camera.read_cv2_image()
            if frame is None:
                continue
            # 识别红色目标
            marked_frame, center = detect_color(frame, red_lower, red_upper, (0, 0, 255))
            # 追踪并控制底盘
            status = track_color_and_move(marked_frame, center, chassis, screen_center)
            cv2.putText(marked_frame, status, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Target Tracking", marked_frame)
            if cv2.waitKey(1) == ord('q'):
                break
    finally:
        chassis.stop()  # 确保停止移动
        cv2.destroyAllWindows()
        camera.stop_video_stream()
        ep_robot.close()
        print("追踪结束")