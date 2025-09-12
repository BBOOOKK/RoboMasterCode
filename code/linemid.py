import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import time

# ======== “少动云台，多动底盘”核心参数 ========
PIXEL_DEADBAND = 8           # 像素死区，小于此误差不转向
Z_CLAMP = 120.0              # 底盘角速度限幅（deg/s）
Z_LPF_ALPHA = 0.70           # 角速度一阶低通，越大越平滑（0~1）
POS_HISTORY = 8              # 线中心滑动平均窗口
CURV_HISTORY = 5             # 曲率滑动平均窗口
CURV_ENTER = 0.50            # 进入“急弯”阈值
CURV_EXIT  = 0.30            # 退出“急弯”阈值（滞回，避免抖动）
X_FAST = 0.80                # 直线基准速度
X_SLOW = 0.50                # 急弯基准速度
SEARCH_Z = 25.0              # 掉线时仅靠底盘慢速旋转（deg/s）
SEARCH_TIMEOUT = 8.0         # 单次寻线超时（秒）
ROI_FROM_BOTTOM = [100, 70, 40]  # 自底向上三条ROI行（像素）
ROI_MIN_PIX = 10             # ROI行最少白点才算有效
SPEED_ERR_SCALE = 0.20       # 偏差对前进速度的抑制强度（0~0.4较合适）

# ======== 增强版PID（沿用你的结构与注释）========
class EnhancedPID:
    def __init__(self, p=0.8, i=0.09, d=0.05, out_limit=80, integral_limit=50):
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
    """
    x_centers: list[float]  成功检测到的各ROI行中心x（像素）
    y_rows:    list[float]  与x_centers一一对应的y（像素）
    返回：曲率代理（斜率缩放），>0右转、<0左转；不足2点或退化情况返回0
    """
    if not x_centers or not y_rows:
        return 0.0
    if len(x_centers) != len(y_rows):
        n = min(len(x_centers), len(y_rows))
        x_centers = x_centers[:n]
        y_rows    = y_rows[:n]
    if len(x_centers) < 2:
        return 0.0

    x = np.asarray(x_centers, dtype=np.float32)
    y = np.asarray(y_rows,    dtype=np.float32)
    if np.unique(y).size < 2 or np.isnan(x).any() or np.isnan(y).any():
        return 0.0

    # 拟合关系：x = m*y + b
    m, b = np.polyfit(y, x, 1)
    return float(m) * 0.5

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def lpf(prev, new, alpha):
    return alpha * prev + (1 - alpha) * new

def deadband(err, band):
    return 0.0 if abs(err) <= band else err

# ======== 初始化硬件 ========
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="sta")

ep_camera = ep_robot.camera
ep_camera.start_video_stream(display=False)

ep_chassis = ep_robot.chassis

# —— 云台锁死：只在启动时定一次角度，之后不再下发任何 gimbal 指令 ——
ep_gimbal = ep_robot.gimbal
ep_gimbal.recenter().wait_for_completed()
ep_gimbal.moveto(pitch=-25, yaw=0, pitch_speed=100, yaw_speed=100).wait_for_completed()
# 可选：如果 SDK 支持挂起姿态随动，可尝试屏蔽自动随动（失败会被忽略）
try:
    if hasattr(ep_gimbal, "suspend"):
        ep_gimbal.suspend()
except Exception:
    pass
# —— 之后全程只动底盘 ——

# 控制器
main_pid   = EnhancedPID(p=0.30, i=0.001, d=0.05, out_limit=Z_CLAMP)   # 常规
curved_pid = EnhancedPID(p=0.60, i=0.002, d=0.08, out_limit=Z_CLAMP)   # 急弯

# 蓝色HSV范围（按现场光照微调）
lower_blue = np.array([100, 80, 50])
upper_blue = np.array([130, 255, 255])

# 动态尺寸与状态
frame_w = None
frame_cx = None

line_lost_count = 0
max_line_lost = 10
searching = False
search_dir = 1
search_t0 = 0.0

pos_hist = []
curv_hist = []
z_lpf_prev = 0.0
in_hard_turn = False

try:
    while True:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        if frame is None:
            continue

        if frame_w is None:
            h, frame_w = frame.shape[:2]
            frame_cx = frame_w // 2
            print(f"检测到图像尺寸: 宽度={frame_w}, 高度={h}")

        # 颜色与形态学
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        h, w = mask.shape
        roi_rows = [clamp(h - dy, 0, h - 1) for dy in ROI_FROM_BOTTOM]

        # 多行取中心，鲁棒估计
        line_centers, line_rows = [], []
        for y in roi_rows:
            row = mask[y, :]
            cols = np.where(row == 255)[0]
            if cols.size > ROI_MIN_PIX:
                cx = int(np.mean(cols))
                line_centers.append(cx)
                line_rows.append(y)
                cv2.line(frame, (0, y), (w, y), (0, 255, 0), 1)
                cv2.circle(frame, (cx, y), 5, (0, 0, 255), -1)

        if line_centers:
            # 找到线，退出寻线状态
            if searching:
                searching = False
                main_pid.clear()
                curved_pid.clear()
                print("找到线条，恢复追踪模式")

            avg_cx = int(np.mean(line_centers))
            pos_hist.append(avg_cx)
            if len(pos_hist) > POS_HISTORY:
                pos_hist.pop(0)
            smoothed_cx = int(np.mean(pos_hist))

            # 曲率估计（用三行点拟合）
            curvature = calculate_curvature(line_centers, line_rows)
            curv_hist.append(curvature)
            if len(curv_hist) > CURV_HISTORY:
                curv_hist.pop(0)
            avg_curv = float(np.mean(curv_hist))

            # 急弯状态机（带滞回）
            if in_hard_turn:
                in_hard_turn = abs(avg_curv) > CURV_EXIT
            else:
                in_hard_turn = abs(avg_curv) > CURV_ENTER

            # 像素偏差（右正左负/或相反都可以，只要与PID一致）
            error_px = frame_cx - smoothed_cx
            error_px = deadband(error_px, PIXEL_DEADBAND)

            # 选择PID & 前进速度
            if in_hard_turn:
                z_cmd = curved_pid.update(0.0, error_px)
                x_base = X_SLOW
            else:
                z_cmd = main_pid.update(0.0, error_px)
                x_base = X_FAST

            # 角速度低通 + 限幅（更稳）
            z_cmd = clamp(lpf(z_lpf_prev, z_cmd, Z_LPF_ALPHA), -Z_CLAMP, Z_CLAMP)
            z_lpf_prev = z_cmd

            # 偏差越大越慢
            x_cmd = x_base - min(abs(error_px) / 100.0, SPEED_ERR_SCALE)
            x_cmd = max(0.0, x_cmd)

            # —— 只动底盘，不动云台 ——
            ep_chassis.drive_speed(x=x_cmd, y=0, z=z_cmd, timeout=0.1)

            # 计数复位与可视化
            line_lost_count = 0
            cv2.circle(frame, (smoothed_cx, h - 50), 8, (255, 0, 0), -1)
            cv2.circle(frame, (frame_cx,   h - 50), 8, (0, 255, 255), -1)
            cv2.putText(frame, f"Err:{int(error_px)}  Z:{z_cmd:.1f}  Curv:{avg_curv:.2f}  {'HARD' if in_hard_turn else 'NORM'}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        else:
            # 未检测到线：仅靠底盘缓慢旋转搜线（云台保持锁死角度）
            line_lost_count += 1
            if line_lost_count > max_line_lost:
                if not searching:
                    searching = True
                    search_t0 = time.time()
                    print("线条丢失，开始寻找...")

                elapsed = time.time() - search_t0
                if elapsed > SEARCH_TIMEOUT:
                    # 反向再找
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
                    search_dir *= -1
                    search_t0 = time.time()
                    cv2.putText(frame, "SEARCH TIMEOUT - DIR FLIP",
                                (w//2 - 150, h//2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                else:
                    ep_chassis.drive_speed(x=0, y=0, z=SEARCH_Z * search_dir, timeout=0.1)
                    cv2.putText(frame, f"SEARCHING... {int(SEARCH_TIMEOUT - elapsed)}s",
                                (w//2 - 120, h//2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

        # 调试画面
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass
finally:
    # 收尾：停止底盘；（云台全程未动，不需要恢复）
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.3)
    time.sleep(0.2)
    ep_camera.stop_video_stream()
    # 可选：若上面调用了 suspend，可尝试恢复（没有该方法会被忽略）
    try:
        if hasattr(ep_gimbal, "resume"):
            ep_gimbal.resume()
    except Exception:
        pass
    ep_robot.close()
    cv2.destroyAllWindows()