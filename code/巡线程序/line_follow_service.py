"""
RoboMaster S1 蓝线巡线接口

功能特性：
- 提供 LineFollower 类：start_background()/run_forever()/stop()/get_status()/update_pid()
- 提供一行式接口：start_line_following(...) -> LineFollower
- 云台锁死（默认 True）：云台复位后固定 pitch=-25°, yaw=0°，只让底盘旋转寻线
- 曲率自适应：急弯自动降速并切换到更“激进”的 PID
- 丢线恢复：超时自动变向寻线
- 事件回调：on_line_found / on_line_lost / on_timeout
- 可选调试画面：叠加中心点、曲率、误差与速度（show_debug=True）

示例（被其他程序调用）：
    from line_follow_service import start_line_following
    lf = start_line_following(show_debug=False)   # 一行启动后台线程
    time.sleep(10)                                # 跑 10 秒
    print(lf.get_status())                        # 查询状态
    lf.stop()                                     # 停止并释放资源

示例（单独运行）：
    if __name__ == "__main__":
        from line_follow_service import LineFollower, LFConfig
        cfg = LFConfig(show_debug=True)
        with LineFollower(cfg) as lf:
            lf.run_forever()  # Ctrl+C 退出
"""

import cv2
import time
import threading
import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional, Tuple, Dict

from robomaster import robot
from robomaster import camera


# ===========================
# 可调参数配置（dataclass）
# ===========================
@dataclass
class LFConfig:
    # 连接/模式
    conn_type: str = "sta"
    chassis_mode = robot.CHASSIS_LEAD  # 底盘随云台模式，但我们锁死云台不转

    # 调试与可视化
    show_debug: bool = False
    debug_window_name: str = "LineFollower"
    debug_mask_name: str = "Mask"

    # 画面与阈值
    lower_blue: Tuple[int, int, int] = (100, 80, 50)
    upper_blue: Tuple[int, int, int] = (130, 255, 255)
    roi_rows_offset: Tuple[int, int, int] = (100, 70, 40)   # 底部多行
    min_white_pixels: int = 10

    # 速度与限幅
    base_speed_straight: float = 0.82
    base_speed_curve: float = 0.58
    z_out_limit_main: float = 80.0
    z_out_limit_curve: float = 100.0

    # 曲率判定
    curvature_scale: float = 0.5   # m * scale
    curvature_enter: float = 0.30
    curvature_exit: float = 0.20
    curvature_history: int = 5

    # 平滑与历史
    position_history: int = 8

    # 寻线/丢线策略
    max_line_lost_frames: int = 10
    search_timeout_s: float = 8.0
    search_speed_deg: float = 20.0

    # 云台（锁死）
    lock_gimbal: bool = True
    gimbal_pitch: float = -25.0
    gimbal_yaw: float = 0.0
    gimbal_speed: int = 100

    # PID 参数
    main_pid_p: float = 0.20
    main_pid_i: float = 0.001
    main_pid_d: float = 0.05
    curve_pid_p: float = 0.50
    curve_pid_i: float = 0.002
    curve_pid_d: float = 0.08

    # 摄像头读取
    read_timeout: float = 0.5


# ===========================
# 增强版 PID 控制器
# ===========================
class EnhancedPID:
    def __init__(self, p=0.75, i=0.09, d=0.05, out_limit=80, integral_limit=50):
        self.kp = p
        self.ki = i
        self.kd = d
        self.out_limit = float(out_limit)
        self.integral_limit = float(integral_limit)
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
        if dt <= 0:
            dt = 0.01

        error = target - current

        # 比例
        p_term = self.kp * error

        # 积分（限幅防风up）
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        i_term = self.ki * self.integral

        # 微分
        d_term = self.kd * (error - self.last_error) / dt
        self.last_error = error

        # 总输出与限幅
        out = p_term + i_term + d_term
        out = max(-self.out_limit, min(out, self.out_limit))
        return out


# ===========================
# 曲率计算
# ===========================
def calculate_curvature(x_centers, y_rows, scale=0.5) -> float:
    """
    x_centers: list[float] ROI各行的线中心x（像素）
    y_rows:    list[float] 一一对应的行 y（像素）
    返回：曲率代理（正右转/负左转），不足2点返回0
    """
    if not x_centers or not y_rows:
        return 0.0
    if len(x_centers) != len(y_rows):
        n = min(len(x_centers), len(y_rows))
        x_centers, y_rows = x_centers[:n], y_rows[:n]
    if len(x_centers) < 2:
        return 0.0

    x = np.asarray(x_centers, dtype=np.float32)
    y = np.asarray(y_rows, dtype=np.float32)
    if np.unique(y).size < 2 or np.isnan(x).any() or np.isnan(y).any():
        return 0.0

    # 拟合 x = m*y + b
    m, _b = np.polyfit(y, x, 1)
    return float(m) * float(scale)


# ===========================
# 巡线服务主体
# ===========================
class LineFollower:
    """
    典型用法：
        with LineFollower(LFConfig(show_debug=True)) as lf:
            lf.start_background()
            time.sleep(8)
            print(lf.get_status())
            lf.stop()

    事件回调：
        lf.on_line_found = lambda status: print("Found:", status)
        lf.on_line_lost  = lambda status: print("Lost:", status)
        lf.on_timeout    = lambda status: print("Timeout:", status)
    """
    def __init__(self, cfg: LFConfig = LFConfig(), ep: Optional[robot.Robot] = None):
        self.cfg = cfg
        self._ep_given = ep is not None

        self.ep_robot: Optional[robot.Robot] = ep
        self.ep_camera = None
        self.ep_chassis = None
        self.ep_gimbal = None

        # 状态
        self.frame_width = None
        self.frame_center = None
        self.line_lost_count = 0
        self.searching = False
        self.search_direction = -1
        self.search_start_time = 0.0

        self.position_history = []
        self.curvature_history = []

        self.avg_curvature = 0.0
        self.last_error = 0.0
        self.last_z_speed = 0.0
        self.last_x_speed = 0.0
        self.last_found = False

        # PID
        self.main_pid = EnhancedPID(
            p=cfg.main_pid_p, i=cfg.main_pid_i, d=cfg.main_pid_d,
            out_limit=cfg.z_out_limit_main
        )
        self.curved_pid = EnhancedPID(
            p=cfg.curve_pid_p, i=cfg.curve_pid_i, d=cfg.curve_pid_d,
            out_limit=cfg.z_out_limit_curve
        )

        # 回调
        self.on_line_found: Optional[Callable[[Dict], None]] = None
        self.on_line_lost: Optional[Callable[[Dict], None]] = None
        self.on_timeout: Optional[Callable[[Dict], None]] = None

        # 线程控制
        self._running = False
        self._th: Optional[threading.Thread] = None

    # ---------- 资源管理 ----------
    def __enter__(self):
        self._initialize_robot()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        self._cleanup()

    def _initialize_robot(self):
        if self.ep_robot is None:
            self.ep_robot = robot.Robot()
            self.ep_robot.initialize(conn_type=self.cfg.conn_type)

        self.ep_camera = self.ep_robot.camera
        self.ep_chassis = self.ep_robot.chassis
        self.ep_gimbal = self.ep_robot.gimbal

        # 摄像头
        self.ep_camera.start_video_stream(display=False)

        # 云台锁死
        self.ep_gimbal.recenter().wait_for_completed()
        self.ep_gimbal.moveto(
            pitch=self.cfg.gimbal_pitch,
            yaw=self.cfg.gimbal_yaw,
            pitch_speed=self.cfg.gimbal_speed,
            yaw_speed=self.cfg.gimbal_speed
        ).wait_for_completed()

        # 模式
        self.ep_robot.set_robot_mode(mode=self.cfg.chassis_mode)

    def _cleanup(self):
        try:
            if self.ep_chassis:
                self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.2)
            # 停止视频
            if self.ep_camera:
                self.ep_camera.stop_video_stream()
            # 关闭窗口
            if self.cfg.show_debug:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass
        finally:
            if self.ep_robot and not self._ep_given:
                self.ep_robot.close()

    # ---------- 公共接口 ----------
    def start_background(self):
        """后台线程持续运行。"""
        if self._running:
            return
        if self.ep_robot is None:
            self._initialize_robot()
        self._running = True
        self._th = threading.Thread(target=self.run_forever, daemon=True)
        self._th.start()
        return self

    def run_forever(self):
        """阻塞运行，KeyboardInterrupt 中断。"""
        if self.ep_robot is None:
            self._initialize_robot()
        try:
            while self._running or self._running is False:
                # 若未显式 start_background()，第一次调用 run_forever() 时启动运行
                self._running = True
                cont = self.step()
                if not cont:
                    time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        """停止运行并安全停止底盘。"""
        self._running = False
        try:
            if self.ep_chassis:
                self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.3)
            # 云台保持锁死（不再发送 yaw 速度命令）
        except Exception:
            pass

    def get_status(self) -> Dict:
        """返回当前状态摘要，便于外部程序轮询。"""
        return dict(
            frame_center=self.frame_center,
            last_error=self.last_error,
            z_speed=self.last_z_speed,
            x_speed=self.last_x_speed,
            avg_curvature=float(self.avg_curvature),
            line_lost_count=int(self.line_lost_count),
            searching=bool(self.searching),
            search_dir=int(self.search_direction),
        )

    def update_pid(self, main=None, curve=None):
        """动态更新 PID 参数：main=(p,i,d,out_limit), curve=(p,i,d,out_limit)"""
        if main:
            p, i, d, out = main
            self.main_pid.kp, self.main_pid.ki, self.main_pid.kd = p, i, d
            self.main_pid.out_limit = float(out)
            self.main_pid.clear()
        if curve:
            p, i, d, out = curve
            self.curved_pid.kp, self.curved_pid.ki, self.curved_pid.kd = p, i, d
            self.curved_pid.out_limit = float(out)
            self.curved_pid.clear()

    # ---------- 单步处理（供外部循环调用或内部线程调用） ----------
    def step(self) -> bool:
        """处理一帧；返回是否继续运行。"""
        if not self._running:
            return False

        # 读帧
        frame = self.ep_camera.read_cv2_image(strategy="newest", timeout=self.cfg.read_timeout)
        if frame is None:
            return True

        # 初始化画面宽度/中心
        if self.frame_width is None:
            h, w = frame.shape[:2]
            self.frame_width = w
            self.frame_center = w // 2
            if self.cfg.show_debug:
                print(f"[LineFollower] 画面: {w}x{h}, 中心: {self.frame_center}")

        # 颜色空间与掩膜
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.cfg.lower_blue), np.array(self.cfg.upper_blue))

        # 形态学 + 模糊
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        # 多行 ROI
        h, w = mask.shape
        roi_rows = [h - off for off in self.cfg.roi_rows_offset]

        line_centers, line_rows = [], []
        for rr in roi_rows:
            rr = max(0, min(h - 1, rr))
            row = mask[rr, :]
            white = np.where(row == 255)[0]
            if len(white) > self.cfg.min_white_pixels:
                cx = int(np.mean(white))
                line_centers.append(cx)
                line_rows.append(rr)
                if self.cfg.show_debug:
                    cv2.line(frame, (0, rr), (w, rr), (0, 255, 0), 1)
                    cv2.circle(frame, (cx, rr), 5, (0, 0, 255), -1)

        if line_centers:
            # 由“寻线中”->“已找到”触发一次回调
            if self.searching:
                self.searching = False
                self.main_pid.clear()
                if self.on_line_found:
                    self.on_line_found(self.get_status())

            # 平滑中心
            avg_center = int(np.mean(line_centers))
            self.position_history.append(avg_center)
            if len(self.position_history) > self.cfg.position_history:
                self.position_history.pop(0)
            smooth_center = int(np.mean(self.position_history))

            # 曲率
            curv = calculate_curvature(line_centers, line_rows, self.cfg.curvature_scale)
            self.curvature_history.append(curv)
            if len(self.curvature_history) > self.cfg.curvature_history:
                self.curvature_history.pop(0)
            self.avg_curvature = np.mean(self.curvature_history)

            # 偏差（像素）
            error = self.frame_center - smooth_center

            # 曲率滞回切换（防抖）
            use_curve_pid = abs(self.avg_curvature) > self.cfg.curvature_enter
            if use_curve_pid:
                z_speed = self.curved_pid.update(0, error)
                x_speed = self.cfg.base_speed_curve
            else:
                # 若已进入曲率模式，直到低于 exit 才退出
                if abs(self.avg_curvature) < self.cfg.curvature_exit:
                    z_speed = self.main_pid.update(0, error)
                    x_speed = self.cfg.base_speed_straight
                else:
                    z_speed = self.curved_pid.update(0, error)
                    x_speed = self.cfg.base_speed_curve

            # 发送控制（只控底盘；云台锁死，不随动）
            self.ep_chassis.drive_speed(x=x_speed, y=0, z=z_speed, timeout=0.1)

            # 记录状态
            self.last_error = float(error)
            self.last_z_speed = float(z_speed)
            self.last_x_speed = float(x_speed)
            self.line_lost_count = 0
            self.last_found = True

            # 可视化
            if self.cfg.show_debug:
                y_vis = h - 50
                cv2.circle(frame, (smooth_center, y_vis), 8, (255, 0, 0), -1)
                cv2.circle(frame, (self.frame_center, y_vis), 8, (0, 255, 255), -1)
                cv2.putText(frame, f"Err: {error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Z: {z_speed:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Curv: {self.avg_curvature:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # 丢线处理
            self.line_lost_count += 1
            self.last_found = False

            if self.line_lost_count > self.cfg.max_line_lost_frames:
                # 进入寻线
                if not self.searching:
                    self.searching = True
                    self.search_start_time = time.time()
                    if self.on_line_lost:
                        self.on_line_lost(self.get_status())

                elapsed = time.time() - self.search_start_time
                if elapsed > self.cfg.search_timeout_s:
                    # 超时切换方向
                    self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
                    self.search_direction *= -1
                    self.search_start_time = time.time()
                    if self.on_timeout:
                        self.on_timeout(self.get_status())
                    if self.cfg.show_debug:
                        cv2.putText(frame, "SEARCH TIMEOUT - CHANGING DIR",
                                    (w // 2 - 160, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    # 仅底盘缓慢旋转寻找，不转云台（云台锁死）
                    search_z = self.cfg.search_speed_deg * self.search_direction
                    self.ep_chassis.drive_speed(x=0, y=0, z=search_z, timeout=0.1)
                    self.last_z_speed = float(search_z)
                    self.last_x_speed = 0.0
                    if self.cfg.show_debug:
                        cv2.putText(frame, f"SEARCHING... {int(self.cfg.search_timeout_s - elapsed)}s",
                                    (w // 2 - 120, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 调试显示
        if self.cfg.show_debug:
            cv2.imshow(self.cfg.debug_window_name, frame)
            cv2.imshow(self.cfg.debug_mask_name, mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                # 支持按 q 主动停止
                self.stop()

        return True


# ===========================
# 一行式启动接口（给其他程序调用）
# ===========================
def start_line_following(
    show_debug: bool = False,
    conn_type: str = "sta",
    lock_gimbal: bool = True,
    gimbal_pitch: float = -25.0,
    gimbal_yaw: float = 0.0,
    callbacks: Optional[Dict[str, Callable[[Dict], None]]] = None,
    **kwargs
) -> LineFollower:
    """
    其他程序直接调用：
        lf = start_line_following(show_debug=False)
        ...  # do something
        lf.stop()

    参数：
        show_debug: 是否显示调试窗口
        conn_type:  连接方式（ap/sta）
        lock_gimbal/gimbal_pitch/gimbal_yaw: 云台锁死与姿态
        callbacks:  可选事件回调 dict，键：'on_line_found'/'on_line_lost'/'on_timeout'
        kwargs:     其他 LFConfig 字段（同名可覆盖）
    """
    cfg = LFConfig(
        show_debug=show_debug,
        conn_type=conn_type,
        lock_gimbal=lock_gimbal,
        gimbal_pitch=gimbal_pitch,
        gimbal_yaw=gimbal_yaw,
        **kwargs
    )
    lf = LineFollower(cfg)
    lf.__enter__()  # 手动进场景，便于返回可用对象
    if callbacks:
        lf.on_line_found = callbacks.get("on_line_found")
        lf.on_line_lost = callbacks.get("on_line_lost")
        lf.on_timeout = callbacks.get("on_timeout")
    return lf.start_background()


# ===========================
# 直接运行：示例
# ===========================
if __name__ == "__main__":
    # 单独运行演示：Ctrl+C 退出
    cfg = LFConfig(show_debug=True)
    with LineFollower(cfg) as follower:
        follower.run_forever()