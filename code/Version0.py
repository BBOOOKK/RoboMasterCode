# file: line_follow_stable.py
import cv2
import numpy as np
from robomaster import robot, camera
from collections import deque
import time
import math

# ========================= 参数区（按需调） =========================
ROI_RATIO = 0.5          # 仅用下半幅
BIN_BLK   = 21           # 自适应阈值 block size（奇数）
BIN_C     = 5
AREA_MIN  = 300          # 最小有效轮廓面积

# 平滑/滤波
EMA_ALPHA_CX    = 0.3    # 质心 EMA
EMA_ALPHA_ANGLE = 0.25   # 角度 EMA
D_FILTER_HZ     = 6.0    # PID 导数通道低通截止频率(Hz)

# PID（基于归一化误差：dx in [-1,1]，angle in [-pi/2,pi/2]）
PID_X = dict(kp=1.6, ki=0.0, kd=0.12, out=(-150, 150))
PID_A = dict(kp=120.0, ki=0.0, kd=12.0, out=(-200, 200))

# 车速策略
V_BASE = 0.45            # 基准前进 m/s
V_MIN  = 0.18
V_MAX  = 0.70
Z_MAX  = 200             # 最大角速度(deg/s)

# 丢线策略
LOST_BUF_N  = 12
SCAN_Z1     = 50         # 轻微扫描角速
SCAN_Z2     = 120        # 强力扫描角速
# ===================================================================


class PID:
    def __init__(self, kp, ki, kd, out=(-1e9, 1e9), i_limit=None, d_hz=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.omin, self.omax = out
        self.i_sum = 0.0
        self.e_prev = 0.0
        self.t_prev = None
        self.i_limit = i_limit
        # 导数低通：y = y + alpha*(x - y)
        self.d_hz = d_hz
        self.d_state = 0.0

    def reset(self):
        self.i_sum = 0.0
        self.e_prev = 0.0
        self.t_prev = None
        self.d_state = 0.0

    def __call__(self, err, dt):
        if dt <= 0:
            dt = 1e-3

        # 积分与抗饱和
        self.i_sum += err * dt
        if self.i_limit is not None:
            lo, hi = self.i_limit
            self.i_sum = max(lo, min(hi, self.i_sum))

        # 原始导数
        d_raw = (err - self.e_prev) / dt
        self.e_prev = err

        # 导数低通（抗噪）
        if self.d_hz and self.d_hz > 0:
            # 一阶滞后：alpha = 1 - exp(-2*pi*fc*dt)
            alpha = 1.0 - math.exp(-2.0 * math.pi * self.d_hz * dt)
            self.d_state += alpha * (d_raw - self.d_state)
            d = self.d_state
        else:
            d = d_raw

        u = self.kp * err + self.ki * self.i_sum + self.kd * d
        if u < self.omin:
            # 反算抗饱和（back-calculation 简化版）
            leak = 0.3
            self.i_sum += leak * (self.omin - u)
            u = self.omin
        elif u > self.omax:
            leak = 0.3
            self.i_sum += leak * (self.omax - u)
            u = self.omax
        return u


def preprocess(frame):
    h, w = frame.shape[:2]
    y0 = int(h * (1.0 - ROI_RATIO))
    roi = frame[y0:h, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    binm = cv2.adaptiveThreshold(
        255 - gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, BIN_BLK, BIN_C
    )
    binm = cv2.medianBlur(binm, 5)
    binm = cv2.morphologyEx(binm, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return binm, roi, (y0, h, w)


def fit_line_info(binm):
    cnts, _ = cv2.findContours(binm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    cnt = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    if area < AREA_MIN:
        return None
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None
    cx = float(M["m10"] / M["m00"])

    # 用最小二乘直线估计方向
    try:
        vx, vy, x0, y0 = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy = float(vx), float(vy)
    except:
        return None

    # 角度归一到 [-pi/2, pi/2]，避免跳变
    ang = math.atan2(vy, vx)  # [-pi, pi]
    if ang > math.pi/2:
        ang -= math.pi
    elif ang < -math.pi/2:
        ang += math.pi

    return dict(contour=cnt, cx=cx, angle=ang, area=area)


def draw_overlay(roi_color, data, ww, info_text):
    h, w = roi_color.shape[:2]
    cx = int(data["cx"])
    cnt = data["contour"]
    ang = data["angle"]

    cv2.drawContours(roi_color, [cnt], -1, (0, 255, 0), 2)
    cv2.circle(roi_color, (cx, h - 5), 5, (0, 0, 255), -1)
    cv2.line(roi_color, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)

    # 指示角度箭头（从底部中央画出）
    L = 60
    x1, y1 = w // 2, h - 10
    x2 = int(x1 + L * math.cos(ang))
    y2 = int(y1 + L * math.sin(ang))
    cv2.arrowedLine(roi_color, (x1, y1), (x2, y2), (0, 255, 255), 2)

    cv2.putText(roi_color, info_text, (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)


def main():
    ep = robot.Robot()
    ep.initialize(conn_type='sta')

    gimbal = ep.gimbal
    chassis = ep.chassis
    cam = ep.camera
    cam.start_video_stream(display=False, resolution=camera.STREAM_720P)
    gimbal.recenter().wait_for_completed()

    pid_x = PID(**PID_X, i_limit=(-2.0, 2.0), d_hz=D_FILTER_HZ)
    pid_a = PID(**PID_A, i_limit=(-2.0, 2.0), d_hz=D_FILTER_HZ)

    lost_buf = deque(maxlen=LOST_BUF_N)

    # EMA 状态
    cx_ema = None
    ang_ema = None

    t_prev = time.time()
    paused = False

    try:
        print("按 q 退出, r 重置PID, p 暂停/恢复底盘")
        while True:
            frame = cam.read_cv2_image(strategy="newest", timeout=5)
            if frame is None:
                # 无帧时轻微扫描
                if not paused:
                    chassis.drive_speed(0.0, 0.0, SCAN_Z1)
                continue

            t = time.time()
            dt = max(1e-3, t - t_prev)
            t_prev = t

            binm, roi, (y0, hh, ww) = preprocess(frame)
            info = fit_line_info(binm)

            if info is None:
                lost_buf.append(1)
                if not paused:
                    # 丢线分级扫描
                    if len(lost_buf) == lost_buf.maxlen and sum(lost_buf) >= int(0.75 * lost_buf.maxlen):
                        chassis.drive_speed(0.0, 0.0, SCAN_Z2)
                    else:
                        chassis.drive_speed(0.0, 0.0, SCAN_Z1)

                cv2.putText(roi, "LOST", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                # 丢线时轻度泄放积分，避免回线瞬间“甩头”
                pid_x.i_sum *= 0.9
                pid_a.i_sum *= 0.9

            else:
                lost_buf.append(0)
                # EMA 平滑
                cx = info["cx"]
                ang = info["angle"]
                cx_ema = cx if cx_ema is None else (1-EMA_ALPHA_CX) * cx_ema + EMA_ALPHA_CX * cx
                ang_ema = ang if ang_ema is None else (1-EMA_ALPHA_ANGLE) * ang_ema + EMA_ALPHA_ANGLE * ang

                # 归一化误差（像素 -> [-1,1]；右正左负）
                dx = (cx_ema - ww * 0.5) / (ww * 0.5)    # 横向相对误差
                ang_n = float(ang_ema)                   # [-pi/2, pi/2]

                # PID 输出：角速度（deg/s）
                u_x = pid_x(dx, dt)
                u_a = pid_a(ang_n, dt)
                z_cmd = float(u_x + u_a)
                z_cmd = max(-Z_MAX, min(Z_MAX, z_cmd))

                # 速度调度：大偏差/大角度→降速
                err_mag = min(1.0, abs(dx))
                ang_mag = min(1.0, abs(ang_n) / (math.pi/2))
                slow = 1.0 - min(0.8, 0.6 * err_mag + 0.4 * ang_mag)

                v = V_BASE * (0.6 + 0.4 * (1.0 - (0.5 * err_mag + 0.5 * ang_mag)))
                v *= slow
                v = float(max(V_MIN, min(V_MAX, v)))

                if not paused:
                    chassis.drive_speed(x=v, y=0.0, z=z_cmd)

                info_text = f"dx={dx:+.2f} ang={ang_n:+.2f} v={v:.2f} wz={z_cmd:+.1f} dt={dt*1000:.0f}ms"
                draw_overlay(roi, dict(contour=info["contour"], cx=cx_ema, angle=ang_ema, area=info["area"]), ww, info_text)

            # 组合视图
            view = np.vstack((roi, cv2.cvtColor(binm, cv2.COLOR_GRAY2BGR)))
            cv2.imshow("Line Follow (roi/top) + bin(bottom)", view)
            key = (cv2.waitKey(1) & 0xFF)
            if key == ord('q'):
                break
            elif key == ord('r'):
                pid_x.reset(); pid_a.reset()
                cx_ema = None; ang_ema = None
                print("PID 已重置")
            elif key == ord('p'):
                paused = not paused
                if paused:
                    chassis.drive_speed(0, 0, 0)
                    print("已暂停底盘")
                else:
                    print("已恢复底盘")

    finally:
        try:
            chassis.drive_speed(0, 0, 0)
        except:
            pass
        cam.stop_video_stream()
        cv2.destroyAllWindows()
        ep.close()


if __name__ == "__main__":
    main()