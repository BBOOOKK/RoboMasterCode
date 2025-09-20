# 06组FinalTask任务代码
# 最终版2025.09.18
# 数字识别部分

import cv2, os, time, threading
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Optional, List

# ==== 常量（与相机/画面相关）====
IMAGE_WIDTH  = 1280
IMAGE_HEIGHT = 720
H_FOV_DEG = 60.0
V_FOV_DEG = 45.0

TARGET_NUMBERS = {"1", "2", "3", "4", "5"}
CENTER_THRESH = 0.05
CENTER_TIMEOUT_S = 3.0          # ↑ 放宽对中超时
CENTER_STABLE_FRAMES = 2
EMA_ALPHA = 0.6
STEP_GAIN = 1.0
MAX_YAW_STEP_DEG   = 180.0
MAX_PITCH_STEP_DEG = 90.0
MIN_STEP_DEG = 1.0
RATE_LIMIT_S = 0.08

# 方向（必要时自动翻转一次）
SIGN_YAW_DEFAULT   =  1.0
SIGN_PITCH_DEFAULT = -1.0

@dataclass
class NumberConfig:
    stop_distance_m: float = 2.0        # 到该距离触发停车对中
    distance_calib_k: float = 0.25      # 距离标定系数（K = d*h）
    stabilize_before_shoot_s: float = 0.30
    stop_sequence_duration_s: float = 2.0
    save_dir: str = "recognized_numbers"

    # 与其他模块的“冲突避免”
    control_drive: bool = True          # True：本模块直接刹车/恢复
    control_gimbal: bool = True         # True：本模块负责对中与回中

class MarkerInfo:
    def __init__(self, x, y, w, h, info):
        self._x, self._y, self._w, self._h = x, y, w, h
        self._info = info
    @property
    def text(self): return self._info
    @property
    def pt1(self):  return int((self._x - self._w/2)*IMAGE_WIDTH),  int((self._y - self._h/2)*IMAGE_HEIGHT)
    @property
    def pt2(self):  return int((self._x + self._w/2)*IMAGE_WIDTH),  int((self._y + self._h/2)*IMAGE_HEIGHT)
    @property
    def center(self):return int(self._x*IMAGE_WIDTH), int(self._y*IMAGE_HEIGHT)

def _to_str(info) -> str:
    if isinstance(info, (bytes, bytearray)):
        try:    return info.decode("utf-8", errors="ignore")
        except (UnicodeDecodeError, AttributeError): 
            return str(info)
    return str(info)

def _clamp(v, lo, hi): return max(lo, min(hi, v))

def _spin_wait(seconds: float):
    end = time.time() + seconds
    while time.time() < end:
        try: 
            cv2.waitKey(1)
        except (cv2.error, RuntimeError):
            # OpenCV窗口可能已关闭，继续等待
            pass
        time.sleep(0.01)

class NumberController:
    def __init__(self, cfg: NumberConfig):
        self.cfg = cfg

        # 机器人句柄（由 bind() 注入）
        self._gimbal = None
        self._chassis = None
        self._led = None

        # 从主循环注入最近一帧：set_frame_provider(lambda: frame)
        self._frame_provider: Optional[Callable[[], Optional[any]]] = None

        # 外部仲裁：红灯等更高优先级 hold
        self._external_hold_checker: Optional[Callable[[], bool]] = None
        self._resume_drive_callback: Optional[Callable[[], None]] = None

        # 并发状态
        self._markers: List[MarkerInfo] = []
        self._mark_lock = threading.Lock()
        self._worker_th: Optional[threading.Thread] = None
        self._running = False

        # 对中/跟踪状态
        self._gimbal_moving = False
        self._target_number: Optional[int] = None
        self._centering_started_at = 0.0
        self._last_move_time = 0.0

        self._ema_err_x = None
        self._ema_err_y = None
        self._last_err_x = None
        self._last_err_y = None
        self._stable_center_count = 0
        self._yaw_flip_done = False
        self._pitch_flip_done = False
        self._sign_yaw = SIGN_YAW_DEFAULT
        self._sign_pitch = SIGN_PITCH_DEFAULT

        self._driving_active = True       # ← 默认认为在行驶，才能确保首次触发“停”
        self._braked_this_seq = False
        self._sequence_started_at = 0.0

        self._recognized = set()
        self._saved = set()
        self._total_saves = 0

    # ========== 外部注入接口 ==========
    def bind(self, gimbal=None, chassis=None, led=None):
        self._gimbal, self._chassis, self._led = gimbal, chassis, led

    def set_frame_provider(self, provider: Callable[[], Optional[any]]):
        self._frame_provider = provider

    def set_external_hold_checker(self, fn: Callable[[], bool]):
        self._external_hold_checker = fn

    def set_resume_drive_callback(self, fn: Callable[[], None]):
        self._resume_drive_callback = fn

    # ========== 并行协调接口 ==========
    def is_busy(self) -> bool:
        """对中/拍照序列期间返回 True，供上层跳过巡线速度下发"""
        if self._gimbal_moving:
            return True
        # 若已经刹停但还没完成整个序列，也视为 busy
        if self.cfg.control_drive and (not self._driving_active):
            return True
        return False

    # ========== 生命周期 ==========
    def start(self):
        if self._worker_th and self._worker_th.is_alive():
            return
        self._running = True
        self._worker_th = threading.Thread(target=self._worker, daemon=True)
        self._worker_th.start()

    def stop(self, timeout: float = 2.0):
        """安全停止线程，等待线程正常结束"""
        self._running = False
        if self._worker_th and self._worker_th.is_alive():
            self._worker_th.join(timeout=timeout)
            if self._worker_th.is_alive():
                print(f"[Number] 警告：工作线程在 {timeout} 秒后仍未停止，可能需要强制终止")
        self._worker_th = None
        print("[Number] 模块已安全停止")

    # ========== 视觉回调（统一订阅一次，在上层转发过来）==========
    def on_detect_marker(self, marker_info):
        with self._mark_lock:
            self._markers.clear()
            if not marker_info:
                return
            for i in range(len(marker_info)):
                x, y, w, h, info = marker_info[i]
                s = _to_str(info)
                self._markers.append(MarkerInfo(x, y, w, h, s))
                if s in TARGET_NUMBERS:
                    n = int(s)
                    if n not in self._recognized:
                        self._recognized.add(n)
                        print(f"[Number] 新识别到: {n}")

    # ========== 对外：可叠加调试信息 ==========
    def draw_debug(self, frame):
        if frame is None: return frame
        with self._mark_lock:
            marks = list(self._markers)
        for m in marks:
            color = (0,255,0) if self._target_number and str(self._target_number)==m.text else (0,255,255)
            cv2.rectangle(frame, m.pt1, m.pt2, color, 2)
            cv2.putText(frame, (m.pt1[0], max(20, m.pt1[1]-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
        stats = f"识别:{sorted(self._recognized)}  已拍:{sorted(self._saved)}  次数:{self._total_saves}"
        cv2.putText(frame, stats, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        return frame

    # ========== 内部工具 ==========
    def _estimate_distance_m(self, marker: MarkerInfo) -> float:
        h = max(float(getattr(marker, "_h", marker._h)), 1e-4)
        return self.cfg.distance_calib_k / h

    def _set_drive(self, x, y=0.0, z=0.0):
        if self._chassis is None: return
        try:
            self._chassis.drive_speed(x=x, y=y, z=z)
        except (ConnectionError, TimeoutError, AttributeError) as e:
            print(f"[Number] 底盘控制失败: {e}")
        except Exception as e:
            print(f"[Number] 未知底盘错误: {e}")

    def _stop_drive(self):
        if self._chassis is None: return
        try:
            self._chassis.drive_speed(x=0.0, y=0.0, z=0.0)  # 不调用 stop()，避免偶发阻塞
        except (ConnectionError, TimeoutError, AttributeError) as e:
            print(f"[Number] 停止底盘失败: {e}")
        except Exception as e:
            print(f"[Number] 未知停止错误: {e}")

    def _move_gimbal_once(self, yaw_step, pitch_step):
        if self._gimbal is None or not self.cfg.control_gimbal: return
        try:
            act = self._gimbal.move(yaw=yaw_step, pitch=pitch_step)
            try:
                act.wait_for_completed(timeout=0.5)
            except (TimeoutError, ConnectionError) as e:
                print(f"[Number] 云台动作超时: {e}")
            except Exception as e:
                print(f"[Number] 云台等待异常: {e}")
        except (ConnectionError, AttributeError) as e:
            print(f"[Number] 云台控制失败: {e}")
        except Exception as e:
            print(f"[Number] 未知云台错误: {e}")

    def _gimbal_recenter(self):
        """将云台复位到巡线所需的位置：pitch=-25.0, yaw=0.0"""
        if self._gimbal is None or not self.cfg.control_gimbal: return
        try:
            # 移动到巡线所需的云台位置，而不是默认的recenter位置
            act = self._gimbal.moveto(pitch=-25.0, yaw=0.0, pitch_speed=100, yaw_speed=100)
            try:
                act.wait_for_completed(timeout=1.0)
            except (TimeoutError, ConnectionError) as e:
                print(f"[Number] 云台回中超时: {e}")
            except Exception as e:
                print(f"[Number] 云台回中等待异常: {e}")
        except (ConnectionError, AttributeError) as e:
            print(f"[Number] 云台回中控制失败: {e}")
        except Exception as e:
            print(f"[Number] 未知云台回中错误: {e}")

    def _maybe_resume_drive(self):
        if not self.cfg.control_drive: return
        # 若外部仍在 hold（如红灯），则不恢复
        if self._external_hold_checker and self._external_hold_checker():
            return
        if self._resume_drive_callback:
            try: 
                self._resume_drive_callback()
                return
            except (ConnectionError, AttributeError) as e:
                print(f"[Number] 恢复驱动回调失败: {e}")
            except Exception as e:
                print(f"[Number] 未知恢复驱动错误: {e}")
        # 默认：自行以慢速恢复（由上层决定是否要更复杂的策略）
        self._set_drive(0.15, 0.0, 0.0)
        self._driving_active = True
        self._braked_this_seq = False

    # 单步对中：返回 True 表示已稳定居中
    def _step_centering(self, target_x: float, target_y: float) -> bool:
        err_x = target_x - 0.5
        err_y = target_y - 0.5

        if self._ema_err_x is None:
            self._ema_err_x, self._ema_err_y = err_x, err_y
        else:
            self._ema_err_x = EMA_ALPHA*err_x + (1-EMA_ALPHA)*self._ema_err_x
            self._ema_err_y = EMA_ALPHA*err_y + (1-EMA_ALPHA)*self._ema_err_y

        instant_centered = (abs(err_x) < CENTER_THRESH) and (abs(err_y) < CENTER_THRESH)
        self._stable_center_count = self._stable_center_count+1 if instant_centered else 0
        if self._stable_center_count >= CENTER_STABLE_FRAMES:
            return True

        # 自校准：误差变大则翻方向，各轴最多翻一次
        now = time.time()
        if (self._last_err_x is not None) and (now - self._last_move_time > 0.2):
            if (not self._yaw_flip_done) and (abs(err_x) > abs(self._last_err_x)*1.25) and (abs(err_x) > 0.12):
                self._sign_yaw *= -1; self._yaw_flip_done = True
                print(f"[Number] 偏航方向翻转为 {self._sign_yaw:+.0f}")
            if (not self._pitch_flip_done) and (abs(err_y) > abs(self._last_err_y)*1.25) and (abs(err_y) > 0.12):
                self._sign_pitch *= -1; self._pitch_flip_done = True
                print(f"[Number] 俯仰方向翻转为 {self._sign_pitch:+.0f}")

        yaw_step   = self._sign_yaw   * self._ema_err_x * (H_FOV_DEG/2.0) * STEP_GAIN
        pitch_step = self._sign_pitch * self._ema_err_y * (V_FOV_DEG/2.0) * STEP_GAIN
        yaw_step   = _clamp(yaw_step,   -MAX_YAW_STEP_DEG,   MAX_YAW_STEP_DEG)
        pitch_step = _clamp(pitch_step, -MAX_PITCH_STEP_DEG, MAX_PITCH_STEP_DEG)

        if abs(yaw_step)   < MIN_STEP_DEG and abs(err_x) > CENTER_THRESH:  yaw_step   = MIN_STEP_DEG if yaw_step>=0   else -MIN_STEP_DEG
        if abs(pitch_step) < MIN_STEP_DEG and abs(err_y) > CENTER_THRESH:  pitch_step = MIN_STEP_DEG if pitch_step>=0 else -MIN_STEP_DEG

        if time.time() - self._last_move_time < RATE_LIMIT_S:
            self._last_err_x, self._last_err_y = err_x, err_y
            return False

        self._move_gimbal_once(yaw_step, pitch_step)
        self._last_move_time = time.time()
        self._last_err_x, self._last_err_y = err_x, err_y
        return False

    def _save_async(self, bgr, number: int):
        os.makedirs(self.cfg.save_dir, exist_ok=True)
        def _worker(img, n):
            try:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                fn = os.path.join(self.cfg.save_dir, f"number_{n}_{ts}.jpg")
                cv2.imwrite(fn, img)
                print(f"[Number] 已保存：{fn}")
            except Exception as e:
                print(f"[Number] 保存失败：{e}")
        threading.Thread(target=_worker, args=(bgr.copy(), number), daemon=True).start()

    # 工作者线程：纯状态机（不阻塞主程序）
    def _worker(self):
        print("[Number] 模块已启动")
        while self._running:
            # 取一份 markers 快照（快速判断是否有候选）
            with self._mark_lock:
                marks = list(self._markers)

            # 选择“未拍过”的、最居中的一个数字作为候选
            cand = None  # (num, MarkerInfo)
            candidates = []
            for m in marks:
                if m.text in TARGET_NUMBERS and int(m.text) not in self._saved:
                    candidates.append((int(m.text), m))
            if candidates:
                cand = min(candidates, key=lambda t: abs(t[1]._x-0.5) + abs(t[1]._y-0.5))

            # 控制逻辑
            if cand is not None:
                n, mk = cand
                dist = self._estimate_distance_m(mk)

                # 到达距离阈值：立即刹车一次并进入对中序列
                if dist <= self.cfg.stop_distance_m:
                    if self.cfg.control_drive and not self._braked_this_seq:
                        self._stop_drive()
                        self._driving_active = False
                        self._sequence_started_at = time.time()
                        self._braked_this_seq = True
                        print(f"[Number] 数字 {n} 距离≈{dist:.2f} m，已停车准备对中")

                    if not self._gimbal_moving:
                        self._target_number = n
                        self._gimbal_moving = True
                        self._centering_started_at = time.time()
                        # 重置对中状态
                        self._ema_err_x = self._ema_err_y = None
                        self._last_err_x = self._last_err_y = None
                        self._stable_center_count = 0
                        self._yaw_flip_done = self._pitch_flip_done = False
                        self._sign_yaw, self._sign_pitch = SIGN_YAW_DEFAULT, SIGN_PITCH_DEFAULT

                else:
                    # 距离尚远：允许慢速巡航（仅当由本模块接管底盘时）
                    if self.cfg.control_drive and self._driving_active and not self._gimbal_moving:
                        # 不强行改你主循环速度，这里不额外加速
                        pass

            # 停车状态下执行对中/拍照
            if self._gimbal_moving and (self._target_number is not None):
                # ★ 每轮重新抓取最新的 markers，避免用旧快照导致“瞬间丢失”
                with self._mark_lock:
                    fresh_marks = list(self._markers)

                # 查找当前目标
                target = None
                for m in fresh_marks:
                    if m.text == str(self._target_number):
                        target = m
                        break

                # 超时丢失：回中并放弃本次，恢复行驶
                if target is None:
                    if time.time() - self._centering_started_at > CENTER_TIMEOUT_S:
                        print(f"[Number] 目标 {self._target_number} 丢失，取消本次拍照")
                        self._gimbal_moving = False
                        # 回中
                        self._gimbal_recenter()
                        # 恢复行驶（若允许且不被红灯 hold）
                        self._maybe_resume_drive()
                        continue  # 进入下一轮

                else:
                    # 步进对中；稳定后拍照
                    if self._step_centering(target._x, target._y):
                        _spin_wait(self.cfg.stabilize_before_shoot_s)

                        # 正确获取帧（修复此前误写成二次调用的 bug）
                        frame = None
                        if callable(self._frame_provider):
                            try:
                                frame = self._frame_provider()
                            except Exception:
                                frame = None

                        img = frame.copy() if frame is not None else None
                        if img is not None:
                            cv2.rectangle(img, target.pt1, target.pt2, (0,255,0), 3)
                            label = f"Team 06 detects a marker with ID of  {self._target_number}"
                            cv2.putText(img, label, (target.pt1[0], max(24, target.pt1[1]-6)), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 2)

                        if self._target_number not in self._saved and img is not None:
                            self._save_async(img, self._target_number)
                            self._total_saves += 1
                            self._saved.add(self._target_number)
                            print(f"[Number] 完成拍照：{self._target_number}")

                        self._gimbal_recenter()

                        # 保证整个“停-对中-拍-回中”序列时长
                        remain = self.cfg.stop_sequence_duration_s - max(0.0, time.time() - self._sequence_started_at)
                        if remain > 0: _spin_wait(remain)

                        # 结束本次序列，准备恢复
                        self._gimbal_moving = False
                        self._target_number = None
                        self._maybe_resume_drive()

            time.sleep(0.01)  # 释放 GIL，降低 CPU

        print("[Number] 模块已停止")
