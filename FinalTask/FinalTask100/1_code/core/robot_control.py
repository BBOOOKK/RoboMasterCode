# 06组FinalTask任务代码
# 最终版2025.09.18
# 核心部分以及巡线部分

import cv2
import numpy as np
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass

from robomaster import robot
from robomaster import camera
from robomaster import led

import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from traffic_control import TrafficLightDetector, TLConfig, draw_debug
from number_control import NumberConfig, NumberController  # ★ 新增



@dataclass
class LineFollowerConfig:
    """巡线机器人配置参数"""
    # 连接设置
    connection_type: str = "ap"
    
    # 颜色检测设置
    lower_blue: Tuple[int, int, int] = (100, 80, 50)
    upper_blue: Tuple[int, int, int] = (130, 255, 255)
    
    # ROI 区域设置 - 扩大检测范围（40个检测点，间隙一致）
    roi_row_offsets: Tuple[int, ...] = (
        320, 313, 306, 299, 292, 285, 278, 271, 264, 257,
        250, 243, 236, 229, 222, 215, 208, 201, 194, 187,
        180, 173, 166, 159, 152, 145, 138, 131, 124, 117,
        110, 103, 96, 89, 82, 75, 68, 61, 60
    )  # 从远到近（40个检测点，间隙一致）
    min_white_pixels: int = 12
    
    # 未来线路预测设置
    prediction_distance: int = 100  # 预测未来像素距离
    early_curve_threshold: float = 0.15  # 提前检测弯道的曲率阈值
    preemptive_slowdown_factor: float = 0.7  # 提前降速系数
    
    # PID 控制器设置
    main_pid_params: Tuple[float, float, float, float] = (0.2, 0.003, 0.05, 80)
    curved_pid_params: Tuple[float, float, float, float] = (0.8, 0.003, 0.20, 150)  # 弯道PID设置
    
    # 云台设置
    gimbal_pitch: float = -25.0
    gimbal_yaw: float = 0.0
    gimbal_speed: int = 100
    
    # 丢线恢复设置
    max_line_lost_frames: int = 10
    
    # 历史数据设置
    position_history_length: int = 12
    curvature_history_length: int = 8
    
    # 速度设置
    base_speed_straight: float = 0.5
    base_speed_curve: float = 0.2
    
    # 曲率阈值设置
    curvature_enter_threshold: float = 0.2   # 进入弯道模式的阈值
    curvature_exit_threshold: float = 0.1    # 退出弯道模式的阈值


class EnhancedPID:
    
    def __init__(self, p: float = 0.75, i: float = 0.09, d: float = 0.05, 
                 out_limit: float = 80, integral_limit: float = 50):
        self.kp = p
        self.ki = i
        self.kd = d
        self.out_limit = float(out_limit)
        self.integral_limit = float(integral_limit)
        self.clear()
    
    def clear(self) -> None:
        self.set_point = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def update(self, target: float, current: float) -> float:
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 避免除零错误
        if dt <= 0:
            dt = 0.01
        
        error = target - current
        
        # 比例项
        p_term = self.kp * error
        
        # 积分项（带钳位）
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        i_term = self.ki * self.integral
        
        # 微分项
        d_term = self.kd * (error - self.last_error) / dt
        self.last_error = error
        
        # 总输出（带钳位）
        output = p_term + i_term + d_term
        output = max(-self.out_limit, min(output, self.out_limit))
        
        return output

# 巡线核心
class LineFollower:
    
    def __init__(self, config: Optional[LineFollowerConfig] = None):
        self.config = config or LineFollowerConfig()
        self._initialize_components()
        self._initialize_state()
        self._initialize_number_module()  # ★ 新增：初始化数字识别拍照模块
    
    def _initialize_components(self) -> None:
        """初始化机器人组件和连接"""
        print(f"使用 {self.config.connection_type.upper()} 模式连接机器人...")
        self.ep_robot = robot.Robot()
        
        try:
            self.ep_robot.initialize(conn_type=self.config.connection_type)
        except Exception as e:
            print(f"机器人初始化失败: {e}")
            print("请检查:")
            print("1. 机器人是否已开机")
            print("2. 网络连接是否正确")
            print("3. 防火墙设置")
            raise
        
        try:
            self.ep_camera = self.ep_robot.camera
            # 建议使用 720p，以便与 number_control 中的宽高常量一致（如需保留默认，可改回去）
            self.ep_camera.start_video_stream(display=False, resolution="720p")
            
            self.ep_chassis = self.ep_robot.chassis
            self.ep_gimbal = self.ep_robot.gimbal
            self.ep_led = self.ep_robot.led
            
            # 初始化云台到固定位置（直接设置为低头位置，避免先回中再低头的延迟）
            print("初始化云台到低头位置...")
            self.ep_gimbal.moveto(
                pitch=self.config.gimbal_pitch,
                yaw=self.config.gimbal_yaw,
                pitch_speed=self.config.gimbal_speed,
                yaw_speed=self.config.gimbal_speed
            ).wait_for_completed()
            
            self.ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
            print("机器人初始化完成")
            
        except Exception as e:
            print(f"组件初始化失败: {e}")
            self.cleanup()
            raise
        
        # 初始化PID控制器
        self.main_pid = EnhancedPID(
            p=self.config.main_pid_params[0],
            i=self.config.main_pid_params[1],
            d=self.config.main_pid_params[2],
            out_limit=self.config.main_pid_params[3]
        )
        
        self.curved_pid = EnhancedPID(
            p=self.config.curved_pid_params[0],
            i=self.config.curved_pid_params[1],
            d=self.config.curved_pid_params[2],
            out_limit=self.config.curved_pid_params[3]
        )
    
    def _initialize_state(self) -> None:
        self.frame_width: Optional[int] = None
        self.frame_center: Optional[int] = None
        
        self.line_lost_count: int = 0
        self.searching: bool = False
        
        # 搜索状态变量
        self.search_state: str = "forward"  # 搜索状态: forward, turn_right, return_right, turn_left, return_left
        self.search_start_time: float = 0.0  # 搜索开始时间
        self.search_cycle_count: int = 0  # 搜索循环计数
        
        self.position_history: List[int] = []
        self.curvature_history: List[float] = []
        
        self.avg_curvature: float = 0.0
        self.last_error: float = 0.0
        self.last_z_speed: float = 0.0
        self.last_x_speed: float = 0.0

        # 交通灯识别器
        self.tl_detector = TrafficLightDetector()

        # 给数字模块使用的共享帧与"红灯保持"信号
        self._last_frame: Optional[np.ndarray] = None
        self.traffic_hold: bool = False
    
    # 初始化数字识别拍照模块
    def _initialize_number_module(self) -> None:
        # 让数字模块自控刹车/恢复与云台
        self.num_cfg = NumberConfig(
            stop_distance_m=2.0,
            distance_calib_k=0.25,
            stabilize_before_shoot_s=0.30,
            stop_sequence_duration_s=2.0,
            control_drive=True,       # 模块自行“停-拍-回-恢复”（恢复前会尊重红灯仲裁）
            control_gimbal=True
        )
        self.num_ctrl = NumberController(self.num_cfg)
        # 绑定硬件句柄
        self.num_ctrl.bind(gimbal=self.ep_gimbal, chassis=self.ep_chassis, led=self.ep_led)
        # 提供最近一帧（用于截图）
        self.num_ctrl.set_frame_provider(lambda: self._last_frame)
        # 红灯仲裁：红灯期间不允许它恢复行驶
        self.num_ctrl.set_external_hold_checker(lambda: self.traffic_hold)
        # 统一订阅一次 marker，并把回调转发给数字模块
        try:
            self.ep_robot.vision.sub_detect_info(name="marker", callback=self._on_marker)
            print("[Vision] 统一订阅 'marker' 成功")
        except Exception as e:
            print(f"[Vision] 订阅 'marker' 失败：{e}")
    
    # 统一的 marker 回调
    def _on_marker(self, info):
        try:
            self.num_ctrl.on_detect_marker(info)
        except Exception as e:
            print(f"[Vision] marker 回调异常：{e}")
    
    # 弯道断线处理机制：对断线前部分进行二次函数拟合随后按曲线过弯
    @staticmethod
    def calculate_curvature(x_centers: List[int], y_rows: List[int]) -> float:

        if not x_centers or not y_rows or len(x_centers) < 3:
            return 0.0
        
        if len(x_centers) != len(y_rows):
            n = min(len(x_centers), len(y_rows))
            x_centers, y_rows = x_centers[:n], y_rows[:n]
        
        x = np.asarray(x_centers, dtype=np.float32)
        y = np.asarray(y_rows, dtype=np.float32)
        
        if np.unique(y).size < 3 or np.isnan(x).any() or np.isnan(y).any():
            return 0.0
        
        try:
            # 二次函数拟合: x = a*y^2 + b*y + c
            coefficients = np.polyfit(y, x, 2)
            a, b, c = coefficients
            # 曲率近似计算: k ≈ |2a|
            curvature = abs(2 * a)
            return float(curvature)
        except:
            return 0.0
    
    # 巡线丢失处理机制：根据斜率变化找回
    @staticmethod
    def get_curve_direction(x_centers: List[int], y_rows: List[int]) -> str:
        if not x_centers or len(x_centers) < 2:
            return "straight"
        
        # 计算斜率变化趋势
        first_half = x_centers[:len(x_centers)//2]
        second_half = x_centers[len(x_centers)//2:]
        
        if not first_half or not second_half:
            return "straight"
        
        avg_first = np.mean(first_half)
        avg_second = np.mean(second_half)
        
        if avg_second < avg_first - 5:  # 向左偏移
            return "left"
        elif avg_second > avg_first + 5:  # 向右偏移
            return "right"
        else:
            return "straight"
    
    # 巡线核心部分：基于ROI和40个检测点的巡线处理
    def _process_frame(self, frame: np.ndarray) -> Tuple[Optional[List[int]], Optional[List[int]], Optional[float]]:
        if self.frame_width is None:
            height, self.frame_width = frame.shape[:2]
            self.frame_center = self.frame_width // 2
            print(f"检测到图像尺寸: 宽度={self.frame_width}, 高度={height}")
            print(f"图像中心点: {self.frame_center}")
        
        # 转换为HSV并创建蓝色掩膜
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.config.lower_blue), np.array(self.config.upper_blue))
        
        # 应用形态学操作
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        
        # 在多个ROI行中检测线条
        height, width = mask.shape
        roi_rows = [height - offset for offset in self.config.roi_row_offsets]
        
        line_centers = []
        line_rows = []
        future_curvature = None
        
        for roi_row in roi_rows:
            roi_row = max(0, min(height - 1, roi_row))
            row_pixels = mask[roi_row, :]
            white_pixels = np.where(row_pixels == 255)[0]
            
            if len(white_pixels) > self.config.min_white_pixels:
                center_x = int(np.mean(white_pixels))
                line_centers.append(center_x)
                line_rows.append(roi_row)
                
                # 绘制调试信息
                cv2.line(frame, (0, roi_row), (width, roi_row), (0, 255, 0), 1)
                cv2.circle(frame, (center_x, roi_row), 5, (0, 0, 255), -1)
        
        # 未来线路预测 - 检测画面最前方的弯道
        if line_centers and len(line_centers) >= 3:
            # 提取最前方的几个点进行预测
            front_indices = sorted(range(len(line_rows)), key=lambda i: line_rows[i])[:3]
            front_centers = [line_centers[i] for i in front_indices]
            front_rows = [line_rows[i] for i in front_indices]
            
            if len(front_centers) >= 3:
                future_curvature = self.calculate_curvature(front_centers, front_rows)
                
                # 绘制未来预测线
                if future_curvature > self.config.early_curve_threshold:
                    # 预测未来位置
                    future_direction = self.get_curve_direction(front_centers, front_rows)
                    future_x = self._predict_future_position(front_centers, front_rows, self.config.prediction_distance)
                    
                    if future_x is not None:
                        cv2.circle(frame, (int(future_x), front_rows[0] - self.config.prediction_distance), 
                                  8, (255, 255, 0), -1)
                        cv2.putText(frame, f"预测曲率: {future_curvature:.3f}", 
                                  (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        return (line_centers, line_rows, future_curvature) if line_centers else (None, None, None)
    
    # 未来线路位置预测（线性回归）
    def _predict_future_position(self, centers: List[int], rows: List[int], distance: int) -> Optional[float]:
        if len(centers) < 2:
            return None
        
        try:
            # 使用线性回归预测未来位置
            x = np.array(centers, dtype=np.float32)
            y = np.array(rows, dtype=np.float32)
            
            # 拟合直线
            coefficients = np.polyfit(y, x, 1)
            m, b = coefficients
            
            # 预测未来位置
            future_y = min(rows) - distance
            future_x = m * future_y + b
            
            return float(future_x)
        except:
            return None
    
    # 弯道减速方案
    def _calculate_curve_speed(self, curvature: float, base_speed: float) -> float:
        # 根据曲率进行线性减速
        # 曲率越大，减速越多
        speed_reduction = min(0.8, curvature * 2.0)  # 最大减速80%
        return base_speed * (1.0 - speed_reduction)
    
    def _handle_line_detected(self, line_centers: List[int], line_rows: List[int], 
                             future_curvature: Optional[float], frame: np.ndarray) -> None:
        if self.searching:
            self.searching = False
            print("找到线条，恢复追踪模式")
            self.main_pid.clear()
        
        # 计算平滑后的中心位置
        avg_center = int(np.mean(line_centers))
        self.position_history.append(avg_center)
        if len(self.position_history) > self.config.position_history_length:
            self.position_history.pop(0)
        
        smoothed_center = int(np.mean(self.position_history))
        
        # 计算曲率和方向
        curvature = self.calculate_curvature(line_centers, line_rows)
        curve_direction = self.get_curve_direction(line_centers, line_rows)
        self.curvature_history.append(curvature)
        if len(self.curvature_history) > self.config.curvature_history_length:
            self.curvature_history.pop(0)
        
        self.avg_curvature = np.mean(self.curvature_history)
        
        # 计算误差
        error = self.frame_center - smoothed_center
        
        # 基础速度设置
        base_speed = self.config.base_speed_straight
        
        # 提前降速逻辑：如果检测到前方有急弯，提前减速
        if (future_curvature is not None and 
            future_curvature > self.config.early_curve_threshold):
            # 提前降速准备入弯
            base_speed *= self.config.preemptive_slowdown_factor
            print(f"前方检测到弯道，提前降速: {base_speed:.2f}m/s")
        
        # 根据曲率选择合适的PID控制器和速度
        use_curve_pid = abs(self.avg_curvature) > self.config.curvature_enter_threshold
        
        if use_curve_pid:
            z_speed = self.curved_pid.update(0, error)
            # 弯道减速方案：使用base_speed_curve并进一步根据曲率减速
            x_speed = self._calculate_curve_speed(curvature, self.config.base_speed_curve)
            control_mode = "弯道模式"
        else:
            z_speed = self.main_pid.update(0, error)
            x_speed = base_speed  # 使用可能调整后的基础速度
            control_mode = "直线模式"
        
        # ★ 与“数字拍照模块”并行协调：若它正忙（对中/拍照），本帧不下发速度，避免抢指令
        try:
            if hasattr(self, "num_ctrl") and hasattr(self.num_ctrl, "is_busy") and self.num_ctrl.is_busy():
                cv2.putText(frame, "Number: BUSY (centering/shooting)", (10, 270), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                return
        except Exception:
            pass

        # 发送控制命令
        self.ep_chassis.drive_speed(x=x_speed, y=0, z=z_speed, timeout=0.1)
        
        # 更新状态
        self.last_error = float(error)
        self.last_z_speed = float(z_speed)
        self.last_x_speed = float(x_speed)
        self.line_lost_count = 0
        
        # 绘制调试信息
        height, _ = frame.shape[:2]
        cv2.circle(frame, (smoothed_center, height - 50), 8, (255, 0, 0), -1)
        cv2.circle(frame, (self.frame_center, height - 50), 8, (0, 255, 255), -1)
        
        cv2.putText(frame, f"误差: {error}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"X速度: {x_speed:.2f}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Z速度: {z_speed:.1f}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"曲率: {curvature:.3f}", (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"预测曲率: {future_curvature:.3f}" if future_curvature is not None else "预测曲率: 无", 
                   (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(frame, f"模式: {control_mode}", (10, 180), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"方向: {curve_direction}", (10, 210), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # 直道断线：执行搜索逻辑
    def _perform_search(self, frame: np.ndarray) -> None:
        current_time = time.time()
        
        # 初始化搜索状态
        if self.search_start_time == 0.0:
            self.search_start_time = current_time
            self.search_state = "forward"
            self.search_cycle_count = 0
            print("开始搜索模式")
        
        # 计算当前状态持续时间
        state_duration = current_time - self.search_start_time
        
        # 根据搜索状态执行不同的动作
        if self.search_state == "forward":
            # 向前走一小段（0.5秒）
            if state_duration < 0.5:
                self.ep_chassis.drive_speed(x=0.3, y=0, z=0, timeout=0.1)
                cv2.putText(frame, "搜索: 向前移动", (10, 300), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                # 切换到右转状态
                self.search_state = "turn_right"
                self.search_start_time = current_time
                print("向前移动完成，开始右转")
        
        elif self.search_state == "turn_right":
            # 向右转（0.3秒）
            if state_duration < 0.5:
                self.ep_chassis.drive_speed(x=0, y=0, z=30, timeout=0.1)
                cv2.putText(frame, "搜索: 向右转", (10, 300), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                # 切换回正状态
                self.search_state = "return_right"
                self.search_start_time = current_time
                print("右转完成，开始回正")
        
        elif self.search_state == "return_right":
            # 回正（0.3秒）
            if state_duration < 0.3:
                self.ep_chassis.drive_speed(x=0, y=0, z=-30, timeout=0.1)
                cv2.putText(frame, "搜索: 回正", (10, 300), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                # 切换到左转状态
                self.search_state = "turn_left"
                self.search_start_time = current_time
                print("回正完成，开始左转")
        
        elif self.search_state == "turn_left":
            # 向左转（0.3秒）
            if state_duration < 0.3:
                self.ep_chassis.drive_speed(x=0, y=0, z=-30, timeout=0.1)
                cv2.putText(frame, "搜索: 向左转", (10, 300), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                # 切换回正状态
                self.search_state = "return_left"
                self.search_start_time = current_time
                print("左转完成，开始回正")
        
        elif self.search_state == "return_left":
            # 回正（0.3秒）
            if state_duration < 0.3:
                self.ep_chassis.drive_speed(x=0, y=0, z=30, timeout=0.1)
                cv2.putText(frame, "搜索: 回正", (10, 300), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                # 完成一个循环，增加计数并重置状态
                self.search_cycle_count += 1
                self.search_state = "forward"
                self.search_start_time = current_time
                print(f"完成搜索循环 #{self.search_cycle_count}，继续搜索")
        
        # 显示搜索状态
        cv2.putText(frame, f"搜索状态: {self.search_state}", (10, 330), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"搜索循环: {self.search_cycle_count}", (10, 360), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    def _handle_line_lost(self, frame: np.ndarray) -> None:
        self.line_lost_count += 1
        
        if self.line_lost_count > self.config.max_line_lost_frames:
            if not self.searching:
                self.searching = True
                self.search_start_time = 0.0  # 重置搜索开始时间
                print("线条丢失，进入搜索模式")
            
            # 执行搜索模式
            self._perform_search(frame)
            
            # 显示搜索状态
            height, width = frame.shape[:2]
            cv2.putText(frame, "线路丢失 - 搜索中",
                       (width // 2 - 100, height // 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    #紧急停车与安全恢复
    def _emergency_stop(self) -> None:
        print("触发紧急停止机制")
        # 立即停止
        self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
        time.sleep(0.5)
        
        # 小幅反向转向尝试恢复
        recovery_speed = -self.last_z_speed * 0.3
        self.ep_chassis.drive_speed(x=0.3, y=0, z=recovery_speed, timeout=0.3)
        time.sleep(0.3)
        
        # 回到安全速度
        self.ep_chassis.drive_speed(x=0.5, y=0, z=0, timeout=0.5)

    # 交通灯图片保存
    def _save_traffic_light_image(self, frame: np.ndarray, light_color: str) -> None:
        # 创建保存目录
        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        save_dir = os.path.join(base_dir, "traffic_light_images")
        os.makedirs(save_dir, exist_ok=True)
        
        # 生成文件名（包含时间戳和颜色）
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"traffic_light_{light_color}_{timestamp}.jpg"
        filepath = os.path.join(save_dir, filename)
        
        # 保存图像
        cv2.imwrite(filepath, frame)
        print(f"已保存交通灯图像: {filepath}")
    
    #主模块
    def run(self) -> None:
        # 启动数字识别模块（后台）
        try:
            self.num_ctrl.start()
        except Exception as e:
            print(f"[Number] 启动失败：{e}")
        try:
            while True:
                frame = self.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
                if frame is None:
                    continue

                # 更新共享帧（供数字模块截图使用）
                self._last_frame = frame

                # 交通灯识别
                color, info = self.tl_detector.detect(frame)

                # 将红灯保持写给数字模块仲裁器
                self.traffic_hold = (color == "red")

                # 添加标注文字和拍照功能
                if color == "red":
                    # 立即停车，并等待直到看到绿灯
                    self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
                    # 设置所有LED为红色
                    self.ep_led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_ON)
                    try:
                        draw_debug(frame, info, "RED")  # 叠画 ROI & 面积数值（可注释）
                    except Exception:
                        pass
                    cv2.putText(frame, "TRAFFIC: RED - STOP", (10, 240),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    # 添加Team 06标注在圆圈下方
                    if info.get("red_cnt") is not None:
                        # 计算圆圈中心位置
                        (x, y), radius = cv2.minEnclosingCircle(info["red_cnt"])
                        x1, y1, x2, y2 = info["roi"]
                        center = (int(x) + x1, int(y) + y1)  # 转换为全局坐标
                        # 在圆圈下方添加文字
                        text_pos = (center[0] - 150, center[1] + int(radius) + 30)
                        cv2.putText(frame, "Team 06 detects a red light", text_pos,
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    else:
                        # 如果没有检测到圆圈，在默认位置显示
                        cv2.putText(frame, "Team 06 detects a red light", (10, 270),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    # 拍照保存
                    self._save_traffic_light_image(frame, "red")
                    
                    # 进入等待绿灯循环
                    print("检测到红灯，停车等待绿灯...")
                    waiting_for_green = True
                    while waiting_for_green:
                        # 读取新帧
                        frame = self.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
                        if frame is None:
                            continue
                            
                        # 更新共享帧
                        self._last_frame = frame
                        
                        # 检测交通灯状态
                        color, info = self.tl_detector.detect(frame)
                        
                        # 显示当前帧
                        try:
                            draw_debug(frame, info, color.upper())
                        except Exception:
                            pass
                            
                        if color == "green":
                            # 检测到绿灯，退出等待循环
                            waiting_for_green = False
                            print("检测到绿灯，恢复行驶")
                            # 设置所有LED为绿色
                            self.ep_led.set_led(comp=led.COMP_ALL, r=0, g=255, b=0, effect=led.EFFECT_ON)
                            
                            # 确保绿灯有圆圈描边
                            try:
                                draw_debug(frame, info, "GREEN")  # 确保绘制绿灯的圆圈
                            except Exception:
                                pass
                                
                            # 添加Team 06标注在圆圈下方
                            if info.get("green_cnt") is not None:
                                # 计算圆圈中心位置
                                (x, y), radius = cv2.minEnclosingCircle(info["green_cnt"])
                                x1, y1, x2, y2 = info["roi"]
                                center = (int(x) + x1, int(y) + y1)  # 转换为全局坐标
                                # 在圆圈下方添加文字
                                text_pos = (center[0] - 150, center[1] + int(radius) + 30)
                                cv2.putText(frame, "Team 06 detects a green light", text_pos,
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            else:
                                # 如果没有检测到圆圈，在默认位置显示
                                cv2.putText(frame, "Team 06 detects a green light", (10, 270),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            
                            # 拍照保存绿灯图像
                            self._save_traffic_light_image(frame, "green")
                        elif color == "red":
                            # 仍然是红灯，继续等待
                            cv2.putText(frame, "等待绿灯中...", (10, 300),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        else:
                            # 没有检测到交通灯，继续等待
                            cv2.putText(frame, "等待绿灯中...", (10, 300),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
                        
                        # 显示图像
                        cv2.imshow("图像帧", frame)
                        
                        # 检查退出键
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            waiting_for_green = False
                            break
                    
                    # 如果用户按q退出，则跳出主循环
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    continue

                elif color == "green":
                    # 设置所有LED为绿色
                    self.ep_led.set_led(comp=led.COMP_ALL, r=0, g=255, b=0, effect=led.EFFECT_ON)
                    try:
                        draw_debug(frame, info, "GREEN")  # 叠画 ROI & 面积数值（可注释）
                    except Exception:
                        pass
                    cv2.putText(frame, "TRAFFIC: GREEN", (10, 240),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # 添加Team 06标注在圆圈下方
                    if info.get("green_cnt") is not None:
                        # 计算圆圈中心位置
                        (x, y), radius = cv2.minEnclosingCircle(info["green_cnt"])
                        x1, y1, x2, y2 = info["roi"]
                        center = (int(x) + x1, int(y) + y1)  # 转换为全局坐标
                        # 在圆圈下方添加文字
                        text_pos = (center[0] - 150, center[1] + int(radius) + 30)
                        cv2.putText(frame, "Team 06 detects a green light", text_pos,
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        # 如果没有检测到圆圈，在默认位置显示
                        cv2.putText(frame, "Team 06 detects a green light", (10, 270),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # 拍照保存
                    self._save_traffic_light_image(frame, "green")

                else:
                    # 没有检测到交通灯时关闭LED
                    self.ep_led.set_led(comp=led.COMP_ALL, r=0, g=0, b=0, effect=led.EFFECT_OFF)
                    cv2.putText(frame, "TRAFFIC: NONE", (10, 240),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

                # 若数字模块当前忙（对中/拍照），本帧跳过巡线控制，避免抢指令
                try:
                    if hasattr(self, "num_ctrl") and hasattr(self.num_ctrl, "is_busy") and self.num_ctrl.is_busy():
                        # 可选：叠加数字模块的调试信息（框、已拍列表等）
                        try:
                            frame = self.num_ctrl.draw_debug(frame)
                        except Exception:
                            pass
                        cv2.imshow("图像帧", frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                        continue
                except Exception:
                    pass
                
                line_centers, line_rows, future_curvature = self._process_frame(frame)
                
                if line_centers:
                    self._handle_line_detected(line_centers, line_rows, future_curvature, frame)
                else:
                    self._handle_line_lost(frame)
                
                # 显示调试窗口
                try:
                    frame_dbg = self.num_ctrl.draw_debug(frame.copy())
                except Exception:
                    frame_dbg = frame
                cv2.imshow("图像帧", frame_dbg)
                cv2.imshow("掩膜(灰度占位)", cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()
    
    def cleanup(self) -> None:
        """清理资源并停止机器人"""
        try:
            # 停止数字模块线程
            try:
                if hasattr(self, "num_ctrl"):
                    self.num_ctrl.stop()
            except Exception:
                pass

            # 退订 marker 回调
            try:
                self.ep_robot.vision.unsub_detect_info(name="marker")
            except Exception:
                pass

            self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
            time.sleep(0.5)
            self.ep_camera.stop_video_stream()
            self.ep_robot.close()
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"清理过程中出错: {e}")
