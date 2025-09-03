from robomaster import robot
import cv2

if __name__ == '__main__':
    ep_robot = robot.Robot(ip="192.168.10.2")
    ep_robot.initialize()
    
    # 开启摄像头和标签识别功能
    camera = ep_robot.camera
    camera.start_video_stream(display=False)
    # 启用AprilTag识别（需机器人支持，部分型号默认支持）
    ep_robot.vision.enable_detection(robot.VISION_DETECTION_APRILTAG)
    
    print("开始AprilTag识别，按 'q' 退出...")
    try:
        while True:
            frame = camera.read_cv2_image()
            if frame is None:
                continue
            # 获取标签识别结果
            april_tags = ep_robot.vision.get_detection_result()
            
            # 在图像上标记标签
            for tag in april_tags:
                # 标签信息：id（编号）、x/y（中心坐标）、w/h（宽度/高度）、yaw（旋转角）
                tag_id = tag[0]
                x, y = tag[1], tag[2]
                w, h = tag[3], tag[4]
                yaw = tag[5]
                
                # 绘制标签边框
                cv2.rectangle(frame, (int(x-w/2), int(y-h/2)), 
                              (int(x+w/2), int(y+h/2)), (255, 0, 0), 2)
                # 显示标签信息
                cv2.putText(frame, f"ID: {tag_id}, Yaw: {yaw:.1f}", 
                            (int(x-w/2), int(y-h/2)-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            cv2.imshow("AprilTag Detection", frame)
            if cv2.waitKey(1) == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()
        camera.stop_video_stream()
        ep_robot.vision.disable_detection()  # 关闭识别功能
        ep_robot.close()