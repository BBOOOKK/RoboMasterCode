import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import time


# PID控制
class EnhancedPID:
    def __init__(self, p=0.75, i=0.09, d=0.05, out_limit=80, integral_limit=50):
        self.kp = p
        self.ki = i
        self.kd = d
        self.out_limit = out_limit
        self.integral_limit = integral_limit
        self.clear()

    def clear(self):
        self.set_point = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def update(self, target, current):
        # 计算时间差
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 避免除零错误
        if dt <= 0:
            dt = 0.01

        error = target - current

        # 比例项
        p_term = self.kp * error

        # 积分项（带积分限幅）
        self.integral += error * dt
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
        i_term = self.ki * self.integral

        # 微分项
        d_term = self.kd * (error - self.last_error) / dt
        self.last_error = error

        # 总输出
        output = p_term + i_term + d_term

        # 输出限制
        if output > self.out_limit:
            output = self.out_limit
        elif output < -self.out_limit:
            output = -self.out_limit

        return output


def calculate_curvature(x_centers, y_rows):

    # 基本校验
    if not x_centers or not y_rows:
        return 0.0
    if len(x_centers) != len(y_rows):
        # 防御式处理（防止曲率收集错误）
        n = min(len(x_centers), len(y_rows))
        x_centers = x_centers[:n]
        y_rows    = y_rows[:n]

    if len(x_centers) < 2:
        return 0.0

    x = np.asarray(x_centers, dtype=np.float32)
    y = np.asarray(y_rows,    dtype=np.float32)

    # y 全相等会导致拟合不稳定
    if np.unique(y).size < 2 or np.isnan(x).any() or np.isnan(y).any():
        return 0.0

    # 拟合关系：x = m*y + b
    m, b = np.polyfit(y, x, 1)
    return float(m) * 0.5


# 初始化机器人
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="sta")

# 初始化摄像头
ep_camera = ep_robot.camera
ep_camera.start_video_stream(display=False)

# 初始化底盘
ep_chassis = ep_robot.chassis

# 初始化云台并设置固定角度（不要控制云台转向）
ep_gimbal = ep_robot.gimbal
ep_gimbal.recenter().wait_for_completed()
ep_gimbal.moveto(pitch=-25, yaw=0, pitch_speed=100, yaw_speed=100).wait_for_completed()
ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)

# 初始化PID控制器
# 主PID用于正常追踪
main_pid = EnhancedPID(p=0.2, i=0.001, d=0.05, out_limit=80)
# 备用PID用于曲率较大时
curved_pid = EnhancedPID(p=0.5, i=0.002, d=0.08, out_limit=100)

# 蓝色HSV范围（需要根据实际环境调整）
lower_blue = np.array([100, 80, 50])  # 提高了饱和度最小值，减少干扰
upper_blue = np.array([130, 255, 255])

# 动态获取图像尺寸
frame_width = None
frame_center = None

# 状态变量
line_lost_count = 0
max_line_lost = 10  # 连续丢失蓝线的最大帧数
searching = False  # 是否正在寻找线条
search_direction = -1  # 1表示顺时针，-1表示逆时针
search_start_time = 0
search_timeout = 8  # 寻找线条的超时时间（秒）

# 历史位置记录（用于平滑处理）
position_history = []
history_length = 8

# 曲率历史记录
curvature_history = []
curvature_history_length = 5

try:
    while True:
        # 获取图像帧
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        if frame is None:
            continue

        # 如果是第一次获取帧，初始化宽度和中心点
        if frame_width is None:
            height, frame_width = frame.shape[:2]
            frame_center = frame_width // 2
            print(f"检测到图像尺寸: 宽度={frame_width}, 高度={height}")
            print(f"图像中心点: {frame_center}")

        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 创建蓝色掩膜
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 形态学操作（去除噪声）
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 先闭运算填充小孔
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # 再开运算去除小点

        # 高斯模糊减少噪声
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        # 选择多个ROI行来提高稳定性
        height, width = mask.shape
        roi_rows = [height - 100, height - 70, height - 40]  # 底部多行检测

        line_centers = []
        line_rows = []  # 新增：与 line_centers 一一对应的行号（y）

        for roi_row in roi_rows:
            # 防止越界（极端分辨率下的保护）
            roi_row = max(0, min(height - 1, roi_row))

            # 提取该行的所有像素
            roi = mask[roi_row, :]

            # 找到白色像素的位置
            white_pixels = np.where(roi == 255)[0]

            if len(white_pixels) > 10:  # 只有足够的像素才认为是有效的线
                # 计算白色像素的中心点
                line_center = int(np.mean(white_pixels))
                line_centers.append(line_center)
                line_rows.append(roi_row)  # 只在成功检测时记录对应行号

                # 在图像上绘制参考线和中点（用于调试）
                cv2.line(frame, (0, roi_row), (width, roi_row), (0, 255, 0), 1)
                cv2.circle(frame, (line_center, roi_row), 5, (0, 0, 255), -1)

        # 计算平均中心点
        if line_centers:
            # 如果之前在寻找线条，现在找到了，重置寻找状态
            if searching:
                searching = False
                print("找到线条，恢复追踪模式")
                main_pid.clear()  # 重置PID控制器

            avg_line_center = int(np.mean(line_centers))

            # 使用历史数据平滑处理
            position_history.append(avg_line_center)
            if len(position_history) > history_length:
                position_history.pop(0)

            smoothed_center = int(np.mean(position_history))

            # 计算线条曲率（改为使用 line_rows 与 line_centers 成对数据）
            curvature = calculate_curvature(line_centers, line_rows)

            # 记录曲率历史
            curvature_history.append(curvature)
            if len(curvature_history) > curvature_history_length:
                curvature_history.pop(0)

            # 计算平均曲率
            avg_curvature = np.mean(curvature_history)

            # 计算与图像中心的偏差
            error = frame_center - smoothed_center

            # 根据曲率选择合适的PID控制器
            if abs(avg_curvature) > 0.3:  # 曲率较大时
                z_speed = curved_pid.update(0, error)
                # 曲率大时降低前进速度
                base_speed = 0.58
            else:  # 正常情况
                z_speed = main_pid.update(0, error)
                base_speed = 0.9

            # 根据偏差大小调整前进速度
            #speed_reduction = min(abs(error) / 100.0, 0.1)  # 偏差越大，速度越慢
            x_speed = base_speed - 0

            # 只控制底盘，不控制云台
            ep_chassis.drive_speed(x=x_speed, y=0, z=z_speed, timeout=0.1)
            ep_gimbal.drive_speed(0, z_speed)

            # 重置丢失计数器
            line_lost_count = 0

            # 在图像上绘制中心点
            cv2.circle(frame, (smoothed_center, height - 50), 8, (255, 0, 0), -1)
            cv2.circle(frame, (frame_center, height - 50), 8, (0, 255, 255), -1)

            # 显示调试信息
            cv2.putText(frame, f"Error: {error}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Z-Speed: {z_speed:.1f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Curvature: {avg_curvature:.2f}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # 没有检测到蓝线
            line_lost_count += 1

            if line_lost_count > max_line_lost:
                # 连续多帧未检测到蓝线，进入寻线模式
                if not searching:
                    searching = True
                    search_start_time = time.time()
                    print("线条丢失，开始寻找...")

                # 检查寻线是否超时
                elapsed_time = time.time() - search_start_time
                if elapsed_time > search_timeout:
                    # 超时，停止并改变方向
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
                    ep_gimbal.drive_speed(0, 0)
                    search_direction *= -1  # 改变方向
                    search_start_time = time.time()  # 重置计时器
                    cv2.putText(frame, "SEARCH TIMEOUT - CHANGING DIR",
                                (width // 2 - 150, height // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    # 缓慢旋转寻找线条
                    search_speed = 20 * search_direction  # 旋转速度
                    ep_chassis.drive_speed(x=0, y=0, z=search_speed, timeout=0.1)
                    ep_gimbal.drive_speed(0, search_speed)
                    cv2.putText(frame, f"SEARCHING... {int(search_timeout - elapsed_time)}s",
                                (width // 2 - 100, height // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                # 短暂丢失，保持上次的控制指令
                pass

        # 显示图像（用于调试）
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        # 按'q'退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass
finally:
    # 停止机器人并清理资源
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
    ep_gimbal.drive_speed(0, 0)
    time.sleep(0.5)
    ep_camera.stop_video_stream()
    ep_robot.close()
    cv2.destroyAllWindows()