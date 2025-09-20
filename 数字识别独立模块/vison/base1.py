import cv2
import time
import os
from datetime import datetime
from robomaster import robot

# 配置常量
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
TARGET_NUMBERS = {"1", "2", "3", "4", "5"}
YELLOW_COLOR = (0, 255, 255)  # BGR
BLACK_COLOR = (0, 0, 0)
GREEN_COLOR = (0, 255, 0)

# 相机视场角（度），可按实际设备调整
H_FOV_DEG = 60.0
V_FOV_DEG = 45.0

# 判定“居中”的阈值（归一化坐标偏差）
CENTER_THRESH = 0.05
# 目标丢失超时（秒）
CENTER_TIMEOUT_S = 2.0
STOP_SEQUENCE_DURATION_S = 5.0
STABILIZE_BEFORE_SHOOT_S = 1.0
# 单步控制增益（将像面误差映射到角度的比例）
STEP_GAIN = 1.0
# 每步最大角度，防止过冲
MAX_YAW_STEP_DEG = 180.0
MAX_PITCH_STEP_DEG = 90.0
# 小于该角度则认为太小，给一个最小步长帮助克服静摩擦
MIN_STEP_DEG = 1.0
# 下发云台命令的最小时间间隔（秒），避免过快导致抖动
RATE_LIMIT_S = 0.08
# 误差指数平滑系数（0~1，越大越信任当前帧，越小越平滑）
EMA_ALPHA = 0.6
# 判定“稳定居中”所需连续帧数
CENTER_STABLE_FRAMES = 2

# 方向符号（根据坐标系可能需要翻转）。
# 默认值（-1, -1）通常满足“目标在右侧 -> 正向偏航为负”的情形。
SIGN_YAW = 1.0
SIGN_PITCH = -1.0

# 底盘慢速行驶参数（单位 m/s）
DRIVE_SPEED_SLOW_MPS = 0.5   # 很慢的运行速度

# 停车距离阈值（米）——达到该距离时停车并开始云台居中、拍照
STOP_DISTANCE_M = 1.50        # <- 你可按需要改这个值

# 简易距离估计：使用“识别到的标记在画面中的归一化高度 h”估计距离
# 经验公式：distance ≈ DISTANCE_CALIB_K / h
# 标定方法：在已知距离 d0（米）处，读取一帧的 h0（0~1），取 K = d0 * h0。
# 例如：在 1.0m 处测得 h0=0.25，则 K=0.25，此处将 K 设为 0.25。
DISTANCE_CALIB_K = 0.25     # <- 距离标定系数，按上面方法改

# 全局状态
markers = []                  # 当前帧的标记框列表
recognized_numbers = set()    # 已识别过（见过）的数字
saved_numbers = set()         # 已拍过照的数字（每个数字只拍一次）
total_saves = 0               # 保存次数

# 跟踪状态
gimbal_moving = False         # 是否正在跟踪一个目标
target_number = None          # 当前跟踪的数字
centering_started_at = 0.0    # 本次跟踪开始时间（用于丢失超时）

# 控制与自适应相关状态
last_err_x = None
last_err_y = None
ema_err_x = None
ema_err_y = None
last_move_time = 0.0
adapt_cooldown_until = 0.0
yaw_flip_done = False
pitch_flip_done = False
stable_center_count = 0

# 底盘运行状态（是否在慢速前进）
driving_active = False
sequence_started_at = 0.0


def _to_str(info):
    """将 SDK 回调里的 info 统一转成字符串。"""
    if isinstance(info, (bytes, bytearray)):
        try:
            return info.decode("utf-8", errors="ignore")
        except Exception:
            return str(info)
    return str(info)


def estimate_distance_m(marker):
    """基于标记框的归一化高度估计与相机的距离（米）。"""
    h = max(float(getattr(marker, '_h', 0.0)), 1e-4)
    return DISTANCE_CALIB_K / h


def on_detect_marker(marker_info):
    """视觉回调：只更新标记列表与已识别集合，不直接触发云台动作。"""
    global markers, recognized_numbers

    if not marker_info:
        markers.clear()
        return

    markers.clear()
    for i in range(len(marker_info)):
        x, y, w, h, info = marker_info[i]
        info_str = _to_str(info)
        markers.append(MarkerInfo(x, y, w, h, info_str))
        if info_str in TARGET_NUMBERS:
            num = int(info_str)
            if num not in recognized_numbers:
                recognized_numbers.add(num)
                print(f"识别到新数字: {num}")


class MarkerInfo:
    """为每个识别到的标记提供像素绘制与文本属性。"""
    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info

    @property
    def pt1(self):
        return int((self._x - self._w / 2) * IMAGE_WIDTH), int((self._y - self._h / 2) * IMAGE_HEIGHT)

    @property
    def pt2(self):
        return int((self._x + self._w / 2) * IMAGE_WIDTH), int((self._y + self._h / 2) * IMAGE_HEIGHT)

    @property
    def center(self):
        return int(self._x * IMAGE_WIDTH), int(self._y * IMAGE_HEIGHT)

    @property
    def text(self):
        return self._info


def draw_annotations(image, markers_list):
    """在图像上绘制框和标签。"""
    for m in markers_list:
        box_color = GREEN_COLOR if str(target_number) == m.text else YELLOW_COLOR
        cv2.rectangle(image, m.pt1, m.pt2, box_color, 3)
        label = f"Number: {m.text}"
        # 在目标框上方显示估计距离（米）
        try:
            d = estimate_distance_m(m)
            label += f"  D≈{d:.2f}m"
        except Exception:
            pass
        text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
        text_x = m.center[0] - text_size[0] // 2
        text_y = m.pt1[1] - 10 if m.pt1[1] > 30 else m.pt2[1] + 30
        cv2.rectangle(image, (text_x - 5, text_y - text_size[1] - 5), (text_x + text_size[0] + 5, text_y + 5), box_color, -1)
        cv2.putText(image, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, BLACK_COLOR, 2)
    return image


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def move_gimbal_to_target(ep_gimbal, target_x, target_y):
    """向目标中心移动一步。加入限幅、平滑与一次性符号自校准。
    返回 True 表示已连续若干帧居中，可进行拍照。"""
    global gimbal_moving, last_err_x, last_err_y, ema_err_x, ema_err_y
    global last_move_time, adapt_cooldown_until, yaw_flip_done, pitch_flip_done
    global stable_center_count

    # 当前帧归一化误差（-0.5~0.5）
    err_x = target_x - 0.5
    err_y = target_y - 0.5

    # 指数平滑，降低抖动
    if ema_err_x is None:
        ema_err_x = err_x
        ema_err_y = err_y
    else:
        ema_err_x = EMA_ALPHA * err_x + (1.0 - EMA_ALPHA) * ema_err_x
        ema_err_y = EMA_ALPHA * err_y + (1.0 - EMA_ALPHA) * ema_err_y

    # 是否已经“近似居中”（即时判断，用于计数稳定帧）
    instant_centered = (abs(err_x) < CENTER_THRESH) and (abs(err_y) < CENTER_THRESH)
    if instant_centered:
        stable_center_count += 1
    else:
        stable_center_count = 0

    # 丢失或已足够稳定则无需再动
    if stable_center_count >= CENTER_STABLE_FRAMES:
        return True

    # 方向自校准（最多各轴翻转一次）：误差明显变大才触发
    now = time.time()
    if last_err_x is not None and now > adapt_cooldown_until:
        try:
            if (not yaw_flip_done) and (abs(err_x) > abs(last_err_x) * 1.25) and (abs(err_x) > 0.12):
                globals()['SIGN_YAW'] = -SIGN_YAW
                yaw_flip_done = True
                adapt_cooldown_until = now + 0.8
                print(f"已翻转偏航方向符号，当前 SIGN_YAW={SIGN_YAW:+.0f}")
            if (not pitch_flip_done) and (abs(err_y) > abs(last_err_y) * 1.25) and (abs(err_y) > 0.12):
                globals()['SIGN_PITCH'] = -SIGN_PITCH
                pitch_flip_done = True
                adapt_cooldown_until = now + 0.8
                print(f"已翻转俯仰方向符号，当前 SIGN_PITCH={SIGN_PITCH:+.0f}")
        except Exception:
            pass

    # 将误差映射为角度步长，并做限幅 + 最小步长
    yaw_step = SIGN_YAW * ema_err_x * (H_FOV_DEG / 2.0) * STEP_GAIN
    pitch_step = SIGN_PITCH * ema_err_y * (V_FOV_DEG / 2.0) * STEP_GAIN

    yaw_step = _clamp(yaw_step, -MAX_YAW_STEP_DEG, MAX_YAW_STEP_DEG)
    pitch_step = _clamp(pitch_step, -MAX_PITCH_STEP_DEG, MAX_PITCH_STEP_DEG)

    if abs(yaw_step) < MIN_STEP_DEG and abs(err_x) > CENTER_THRESH:
        yaw_step = MIN_STEP_DEG if yaw_step >= 0 else -MIN_STEP_DEG
    if abs(pitch_step) < MIN_STEP_DEG and abs(err_y) > CENTER_THRESH:
        pitch_step = MIN_STEP_DEG if pitch_step >= 0 else -MIN_STEP_DEG

    # 速率限制：过快下发会造成抖动
    if now - last_move_time < RATE_LIMIT_S:
        last_err_x = err_x
        last_err_y = err_y
        return False

    try:
        ep_gimbal.move(yaw=yaw_step, pitch=pitch_step).wait_for_completed()
        last_move_time = now
    except Exception as e:
        gimbal_moving = False
        print(f"云台移动出错: {e}")
        return False

    # 记录当前误差，供下轮自适应判断
    last_err_x = err_x
    last_err_y = err_y

    return False


def save_number_image(img, number):
    """保存一张带标注的图像到 recognized_numbers 目录（每个数字只保存一次）。"""
    try:
        os.makedirs("recognized_numbers", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fn = f"recognized_numbers/number_{number}_{ts}.jpg"
        cv2.imwrite(fn, img)
        print(f"已保存图片: {fn}")
        return fn
    except Exception as e:
        print(f"保存图片出错: {e}")
        return None


def _set_drive(ep_chassis, x, y=0.0, z=0.0):
    try:
        ep_chassis.drive_speed(x=x, y=y, z=z)
    except Exception:
        pass


def _stop_drive(ep_chassis):
    try:
        ep_chassis.drive_speed(x=0.0, y=0.0, z=0.0)
        ep_chassis.stop()
    except Exception:
        pass


if __name__ == '__main__':
    # 初始化机器人
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_chassis = getattr(ep_robot, 'chassis', None)

    # 允许通过环境变量强制指定方向（YAW_SIGN/PITCH_SIGN ∈ {+1,-1}）
    try:
        _ys = os.environ.get('YAW_SIGN')
        _ps = os.environ.get('PITCH_SIGN')
        if _ys in ('1', '+1', '-1'):
            globals()['SIGN_YAW'] = 1.0 if _ys.startswith('1') or _ys.startswith('+') else -1.0
        if _ps in ('1', '+1', '-1'):
            globals()['SIGN_PITCH'] = 1.0 if _ps.startswith('1') or _ps.startswith('+') else -1.0
    except Exception:
        pass

    # 开启视频与标记识别回调
    ep_camera.start_video_stream(display=False)
    ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)

    # 云台回中，确保起点一致
    ep_gimbal.recenter().wait_for_completed()

    # 启动慢速行驶
    if ep_chassis is not None:
        _set_drive(ep_chassis, DRIVE_SPEED_SLOW_MPS, 0.0, 0.0)
        driving_active = True

    print("数字识别与云台跟踪程序启动（慢速行驶 + 到 0.5m 停车对中拍照）")
    print("可调参数：STOP_DISTANCE_M、DISTANCE_CALIB_K、DRIVE_SPEED_SLOW_MPS")

    try:
        while True:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            if img is None:
                continue

            # 使用快照，避免回调并发修改
            local_markers = list(markers)

            # 选择一个“尚未拍过”的可见数字作为候选，优先最居中
            candidate = None  # (num, marker)
            candidates = []
            for m in local_markers:
                if m.text in TARGET_NUMBERS:
                    num = int(m.text)
                    if num in saved_numbers:
                        continue  # 只拍一次
                    candidates.append((num, m))
            if candidates:
                candidate = min(candidates, key=lambda t: abs(t[1]._x - 0.5) + abs(t[1]._y - 0.5))

            # 控制底盘与云台的简单状态机
            if candidate is not None:
                cand_num, cand_marker = candidate
                dist_m = estimate_distance_m(cand_marker)

                # 如果距离已小于阈值：停车 -> 开始云台对中
                if dist_m <= STOP_DISTANCE_M:
                    if ep_chassis is not None and driving_active:
                        _stop_drive(ep_chassis)
                        driving_active = False
                        sequence_started_at = time.time()
                        print(f"到达目标 {cand_num} 附近（≈{dist_m:.2f}m），停车并准备对中")

                    # 若未进入对中流程，则设定目标并复位控制状态
                    if not gimbal_moving:
                        target_number = cand_num
                        gimbal_moving = True
                        centering_started_at = time.time()
                        # 重置控制状态，避免上一次残留影响本次
                        last_err_x = last_err_y = None
                        ema_err_x = ema_err_y = None
                        yaw_flip_done = pitch_flip_done = False
                        stable_center_count = 0
                else:
                    # 距离尚远：保持/恢复慢速前进
                    if ep_chassis is not None and not driving_active:
                        _set_drive(ep_chassis, DRIVE_SPEED_SLOW_MPS, 0.0, 0.0)
                        driving_active = True

            # 只有在停车状态下才进行云台对中与拍照
            if gimbal_moving and target_number is not None and not driving_active:
                # 查找当前目标对应的标记
                target_marker = None
                for m in local_markers:
                    if m.text == str(target_number):
                        target_marker = m
                        break

                # 目标暂时不见且超时 -> 取消跟踪并回中，恢复行驶
                if target_marker is None:
                    if time.time() - centering_started_at > CENTER_TIMEOUT_S:
                        print(f"目标 {target_number} 丢失，取消本次拍照，继续行驶")
                        gimbal_moving = False
                        target_number = None
                        try:
                            ep_gimbal.recenter().wait_for_completed()
                        except Exception:
                            pass
                        if ep_chassis is not None and not driving_active:
                            _set_drive(ep_chassis, DRIVE_SPEED_SLOW_MPS, 0.0, 0.0)
                            driving_active = True
                else:
                    # 朝目标移动；若连续稳定居中则拍照并回中，随后继续行驶
                    if move_gimbal_to_target(ep_gimbal, target_marker._x, target_marker._y):
                        # Stabilize briefly before shooting
                        try:
                            time.sleep(STABILIZE_BEFORE_SHOOT_S)
                        except Exception:
                            pass
                        annotated = img.copy()
                        annotated = draw_annotations(annotated, local_markers)
                        # 仅当该数字尚未保存过时才保存
                        if target_number not in saved_numbers:
                            fn = save_number_image(annotated, target_number)
                            if fn:
                                total_saves += 1
                                saved_numbers.add(target_number)
                                print(f"已拍照并保存：数字 {target_number}；该数字后续不再拍摄")
                        try:
                            ep_gimbal.recenter().wait_for_completed()
                        except Exception:
                            pass
                        # Ensure the whole stop-center-shoot-return lasts ~STOP_SEQUENCE_DURATION_S
                        try:
                            remain = STOP_SEQUENCE_DURATION_S - max(0.0, time.time() - sequence_started_at)
                            if remain > 0:
                                time.sleep(remain)
                        except Exception:
                            pass
                        gimbal_moving = False
                        target_number = None
                        # 继续慢速行驶
                        if ep_chassis is not None and not driving_active:
                            _set_drive(ep_chassis, DRIVE_SPEED_SLOW_MPS, 0.0, 0.0)
                            driving_active = True

            # 叠加 HUD
            display_img = img.copy()
            display_img = draw_annotations(display_img, local_markers)
            stats_text = f"已识别: {sorted(recognized_numbers)} | 已保存: {sorted(saved_numbers)} | 次数: {total_saves}"
            cv2.putText(display_img, stats_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, GREEN_COLOR, 2)

            cv2.imshow("数字识别与云台跟踪", display_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("退出程序")
                break

    except KeyboardInterrupt:
        print("用户中断程序")
    except Exception as e:
        print(f"运行时异常: {e}")
    finally:
        print("开始清理资源...")
        try:
            ep_vision.unsub_detect_info(name="marker")
        except Exception:
            pass
        try:
            ep_camera.stop_video_stream()
        except Exception:
            pass
        try:
            ep_robot.close()
        except Exception:
            pass
        cv2.destroyAllWindows()
        print("清理完成")
