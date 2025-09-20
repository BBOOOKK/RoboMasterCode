"""
RoboMaster S1 巡线核心控制模块
包含主要的 LineFollower 类和 EnhancedPID 控制器
"""

import cv2
import numpy as np
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass

from robomaster import robot
from robomaster import camera


@dataclass
class LineFollowerConfig:
    """巡线机器人配置参数"""
    # 连接设置
    connection_type: str = "ap"
    
    # 颜色检测设置
    lower_blue: Tuple[int, int, int] = (100, 80, 50)
    upper_blue: Tuple[int, int, int] = (130, 255, 255)
    
    # ROI 区域设置 - 扩大检测范围
    roi_row_offsets: Tuple[int, int, int, int, int] = (160, 120, 90, 60, 30)  # 从远到近
    min_white_pixels: int = 8
    
    # 未来线路预测设置
    prediction_distance: int = 150  # 预测未来像素距离
    early_curve_threshold: float = 0.25  # 提前检测弯道的曲率阈值
    
    
    # PID 控制器设置
    main_pid_params: Tuple[float, float, float, float] = (0.6, 0.0013, 0.17, 160)     # 主PID参数
    curved_pid_params: Tuple[float, float, float, float] = (3.8, 0.018, 0.12, 340)   # 普通弯道PID参数

    # 速度设置
    base_speed_straight: float = 1.5  # 直线基础速度
    base_speed_curve: float = 0.52        # 弯道基础速度

    # 曲率阈值设置
    curvature_enter_threshold: float = 0.05   # 进入弯道阈值
    curvature_exit_threshold: float = 0.04    # 退出弯道阈值
    
    # 云台设置
    gimbal_pitch: float = -25.0
    gimbal_yaw: float = 0.0
    gimbal_speed: int = 100
    
    # 丢线恢复设置
    max_line_lost_frames: int = 10
    search_timeout_seconds: float = 8.0
    search_speed_degrees: float = 20.0
    
    # 历史数据设置
    position_history_length: int = 5          # 位置历史数据点数
    curvature_history_length: int = 3         # 曲率历史数据点数
    


class EnhancedPID:
    """增强型PID控制器，带积分限制和输出钳位"""
    
    def __init__(self, p: float = 0.75, i: float = 0.09, d: float = 0.05, 
                 out_limit: float = 80, integral_limit: float = 50):
        """
        初始化PID控制器
        
        参数:
            p: 比例增益
            i: 积分增益
            d: 微分增益
            out_limit: 输出限制（钳位）
            integral_limit: 积分项限制（抗饱和）
        """
        self.kp = p
        self.ki = i
        self.kd = d
        self.out_limit = float(out_limit)
        self.integral_limit = float(integral_limit)
        self.clear()
    
    def clear(self) -> None:
        """重置PID控制器状态"""
        self.set_point = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def update(self, target: float, current: float) -> float:
        """
        更新PID控制器并返回控制输出
        
        参数:
            target: 目标值
            current: 当前测量值
            
        返回:
            控制输出
        """
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


class LineFollower:
    """主巡线类，采用专业结构"""
    
    def __init__(self, config: Optional[LineFollowerConfig] = None):
        """
        初始化巡线机器人
        
        参数:
            config: 配置参数（如果为None则使用默认值）
        """
        self.config = config or LineFollowerConfig()
        self._initialize_components()
        self._initialize_state()
    
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
            self.ep_camera.start_video_stream(display=False)
            
            self.ep_chassis = self.ep_robot.chassis
            self.ep_gimbal = self.ep_robot.gimbal
            
            # 初始化云台到固定位置
            print("初始化云台...")
            self.ep_gimbal.recenter().wait_for_completed()
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
            # 避免在初始化失败时调用 cleanup()，因为可能还没有完全初始化
            try:
                self.cleanup()
            except:
                pass
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
        """初始化内部状态变量"""
        self.frame_width: Optional[int] = None
        self.frame_center: Optional[int] = None
        
        self.line_lost_count: int = 0
        self.searching: bool = False
        self.search_direction: int = -1
        self.search_start_time: float = 0.0
        
        self.position_history: List[int] = []
        self.curvature_history: List[float] = []
        
        self.avg_curvature: float = 0.0
        self.last_error: float = 0.0
        self.last_curve_direction: str = "straight"  # 记录最后一次弯道方向
        self.last_z_speed: float = 0.0  # 记录最后一次转向速度
        self.last_x_speed: float = 0.0  # 记录最后一次前进速度
        
        # 新增：PID输出平滑滤波
        self.z_speed_filtered: float = 0.0
        self.x_speed_filtered: float = 0.0
        self.filter_alpha: float = 0.3  # 低通滤波系数，值越小越平滑
    
    @staticmethod
    def calculate_curvature(x_centers: List[int], y_rows: List[int]) -> float:
        """
        计算检测到的线条的曲率（二次函数拟合）
        
        参数:
            x_centers: 线条中心点的x坐标列表
            y_rows: 对应的y坐标列表
            
        返回:
            计算出的曲率值 k ≈ |2a|
        """
        if not x_centers or not y_rows or len(x_centers) < 3:
            return 0.0
        
        # 确保x和y长度一致
        n = min(len(x_centers), len(y_rows))
        if n < 3:
            return 0.0
            
        x = np.array(x_centers[:n], dtype=np.float32)
        y = np.array(y_rows[:n], dtype=np.float32)
        
        # 检查数据有效性
        if np.unique(y).size < 3 or np.any(np.isnan(x)) or np.any(np.isnan(y)):
            return 0.0
        
        try:
            # 二次函数拟合: x = a*y^2 + b*y + c
            coefficients = np.polyfit(y, x, 2)
            a = coefficients[0]  # 只需要a系数计算曲率
            return abs(2 * a)  # 曲率近似计算: k ≈ |2a|
        except (np.linalg.LinAlgError, ValueError):
            return 0.0
    
    @staticmethod
    def get_curve_direction(x_centers: List[int], y_rows: List[int]) -> str:
        """
        判断弯道方向
        
        参数:
            x_centers: 线条中心点的x坐标列表
            y_rows: 对应的y坐标列表
            
        返回:
            "left", "right", 或 "straight"
        """
        if not x_centers or len(x_centers) < 2:
            return "straight"
        
        # 使用线性回归判断方向趋势
        n = min(len(x_centers), len(y_rows))
        if n < 2:
            return "straight"
            
        x = np.array(x_centers[:n], dtype=np.float32)
        y = np.array(y_rows[:n], dtype=np.float32)
        
        try:
            # 线性拟合: x = m*y + b
            m, _ = np.polyfit(y, x, 1)
            
            # 根据斜率判断方向
            if m < -0.1:  # 负斜率表示向左转
                return "left"
            elif m > 0.1:  # 正斜率表示向右转
                return "right"
            else:
                return "straight"
        except (np.linalg.LinAlgError, ValueError):
            return "straight"
    
    def _process_frame(self, frame: np.ndarray) -> Tuple[Optional[List[int]], Optional[List[int]], Optional[float]]:
        """
        处理单帧图像以检测蓝线，包含10点预测趋势和未来线路预测
        
        参数:
            frame: 来自摄像头的输入帧
            
        返回:
            (line_centers, line_rows, future_curvature) 元组
        """
        if self.frame_width is None:
            height, self.frame_width = frame.shape[:2]
            self.frame_center = self.frame_width // 2
            print(f"检测到图像尺寸: 宽度={self.frame_width}, 高度={height}")
            print(f"图像中心点: {self.frame_center}")
        
        # 创建蓝色掩膜并进行二值化处理
        mask = self._create_blue_mask(frame)
        
        # 在多个ROI行中检测线条 - 15个检测点（5个大弯预测点 + 10个常规点）
        height, width = mask.shape
        # 大弯预测点：更宽的间距，用于检测大弯趋势（180-240像素）
        sharp_turn_rows = [height - offset for offset in range(180, 241, 15)]  # 5个大弯预测点
        # 常规检测点：密集间距，用于精确跟踪（30-180像素）
        regular_rows = [height - offset for offset in range(30, 181, 15)]  # 10个常规点
        roi_rows = sharp_turn_rows + regular_rows  # 总共15个检测点
        
        line_centers = []
        line_rows = []
        valid_points = 0
        
        for roi_row in roi_rows:
            roi_row = max(0, min(height - 1, roi_row))
            row_pixels = mask[roi_row, :]
            white_pixels = np.where(row_pixels == 255)[0]
            
            if len(white_pixels) > self.config.min_white_pixels:
                center_x = int(np.mean(white_pixels))
                line_centers.append(center_x)
                line_rows.append(roi_row)
                valid_points += 1
                
                # 绘制调试信息
                cv2.line(frame, (0, roi_row), (width, roi_row), (0, 255, 0), 1)
                cv2.circle(frame, (center_x, roi_row), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"{center_x}", (center_x + 5, roi_row), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # 10点趋势预测和未来线路预测
        future_curvature = self._predict_10point_trend(line_centers, line_rows, frame)
        
        # 大弯趋势检测和1.5倍转弯速率控制
        sharp_turn_detected = False
        if len(line_centers) > 5:
            sharp_turn_detected = self._detect_sharp_turn(line_centers[:5], line_rows[:5])
        
        # 显示二值化图像
        binary_display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        for center, row in zip(line_centers, line_rows):
            cv2.circle(binary_display, (center, row), 5, (0, 0, 255), -1)
        
        cv2.putText(binary_display, f"有效点: {valid_points}/15", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 区分大弯预测点和常规点
        if len(line_centers) > 5:
            # 大弯预测点用不同颜色标记
            for center, row in zip(line_centers[:5], line_rows[:5]):
                cv2.circle(binary_display, (center, row), 7, (255, 0, 0), -1)  # 蓝色标记大弯预测点
            cv2.putText(binary_display, "大弯预测点", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        # 显示二值化图像窗口
        cv2.imshow("二值化图像", binary_display)
        
        return (line_centers, line_rows, future_curvature) if line_centers else (None, None, None)
    
    def _predict_10point_trend(self, line_centers: List[int], line_rows: List[int], 
                              frame: np.ndarray) -> Optional[float]:
        """
        10点趋势预测算法 - 提高预测准确性
        
        参数:
            line_centers: 检测到的中心点列表
            line_rows: 对应的行坐标列表
            frame: 用于绘制预测信息的帧
            
        返回:
            预测的未来曲率
        """
        if not line_centers or len(line_centers) < 5:
            return None
        
        # 确保数据长度一致
        n = min(len(line_centers), len(line_rows))
        centers = line_centers[:n]
        rows = line_rows[:n]
        
        try:
            # 使用加权最小二乘法进行趋势预测
            # 给近处的点更高权重
            weights = np.linspace(0.3, 1.0, n)  # 从远处到近处权重递增
            
            # 二次多项式拟合
            x = np.array(centers, dtype=np.float32)
            y = np.array(rows, dtype=np.float32)
            
            # 加权多项式拟合
            coefficients = np.polyfit(y, x, 2, w=weights)
            a, b, c = coefficients
            
            # 计算当前曲率
            current_curvature = abs(2 * a)
            
            # 预测未来趋势 - 使用线性外推
            if n >= 3:
                # 计算最近3个点的斜率趋势
                recent_x = centers[-3:]
                recent_y = rows[-3:]
                slope, intercept = np.polyfit(recent_y, recent_x, 1)
                
                # 预测未来位置
                future_y = min(rows) - self.config.prediction_distance
                future_x = slope * future_y + intercept
                
                # 绘制预测线
                cv2.line(frame, (centers[0], rows[0]), (int(future_x), int(future_y)), 
                        (255, 0, 255), 2)
                cv2.circle(frame, (int(future_x), int(future_y)), 8, (255, 0, 255), -1)
                
                # 显示预测信息
                cv2.putText(frame, f"10点趋势预测: {current_curvature:.3f}", 
                          (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                cv2.putText(frame, f"斜率: {slope:.3f}", 
                          (10, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            
            return current_curvature
            
        except (np.linalg.LinAlgError, ValueError):
            return None
    
    def _detect_sharp_turn(self, sharp_turn_centers: List[int], sharp_turn_rows: List[int]) -> bool:
        """
        检测大弯趋势 - 基于5个大弯预测点
        
        参数:
            sharp_turn_centers: 大弯预测点的中心坐标
            sharp_turn_rows: 大弯预测点的行坐标
            
        返回:
            是否检测到大弯趋势
        """
        if not sharp_turn_centers or len(sharp_turn_centers) < 3:
            return False
        
        # 计算大弯预测点的曲率
        sharp_curvature = self.calculate_curvature(sharp_turn_centers, sharp_turn_rows)
        
        # 大弯阈值判断
        return sharp_curvature > 0.37  # 曲率大于0.37表示大弯
    
    def _create_blue_mask(self, frame: np.ndarray) -> np.ndarray:
        """创建蓝色掩膜并进行形态学处理 - 增强稳定性版本"""
        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 使用自适应阈值处理来提高光照变化下的稳定性
        h, s, v = cv2.split(hsv)
        
        # 对亮度通道进行直方图均衡化，提高对比度
        v_eq = cv2.equalizeHist(v)
        hsv_eq = cv2.merge([h, s, v_eq])
        
        # 创建蓝色掩膜
        mask = cv2.inRange(hsv_eq, np.array(self.config.lower_blue), np.array(self.config.upper_blue))
        
        # 应用更精细的形态学操作
        kernel_close = np.ones((7, 7), np.uint8)  # 更大的闭运算核，连接断裂区域
        kernel_open = np.ones((3, 3), np.uint8)   # 更小的开运算核，去除小噪点
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        
        # 应用高斯模糊和平滑处理
        mask = cv2.GaussianBlur(mask, (7, 7), 0)
        
        # 二值化处理，确保清晰的边缘
        _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
        
        return mask
    
    def _predict_future_curvature(self, line_centers: List[int], line_rows: List[int], 
                                 frame: np.ndarray) -> Optional[float]:
        """预测未来曲率并绘制调试信息"""
        if not line_centers or len(line_centers) < 3:
            return None
        
        # 提取最前方的几个点进行预测
        front_indices = sorted(range(len(line_rows)), key=lambda i: line_rows[i])[:3]
        front_centers = [line_centers[i] for i in front_indices]
        front_rows = [line_rows[i] for i in front_indices]
        
        if len(front_centers) < 3:
            return None
            
        future_curvature = self.calculate_curvature(front_centers, front_rows)
        
        # 仅绘制预测信息，不用于控制决策
        cv2.putText(frame, f"预测曲率: {future_curvature:.3f}", 
                  (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        return future_curvature
    
    def _handle_line_detected(self, line_centers: List[int], line_rows: List[int], 
                             future_curvature: Optional[float], frame: np.ndarray) -> None:
        """
        处理检测到线条的情况 - 弯道减速方案
        
        参数:
            line_centers: 检测到的线条中心位置
            line_rows: 对应的行位置
            future_curvature: 预测的未来曲率
            frame: 当前帧（用于调试）
        """
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
        self.last_curve_direction = curve_direction  # 记录最后一次弯道方向
        
        # 计算误差
        error = self.frame_center - smoothed_center
        
        # 基础速度设置
        base_speed = self.config.base_speed_straight
        
        # 清理提前降速逻辑，避免代码冲突
        # 未来曲率预测仅用于显示，不用于控制决策
        
        # 简化控制逻辑：只使用基础PID控制器
        use_curve_pid = abs(self.avg_curvature) > self.config.curvature_enter_threshold
        
        if use_curve_pid:
            # 弯道模式：使用弯道PID控制器
            z_speed = self.curved_pid.update(0, error)
            x_speed = self.config.base_speed_curve  # 使用基础弯道速度
            control_mode = "弯道模式"
        else:
            # 直线模式：使用主PID控制器
            z_speed = self.main_pid.update(0, error)
            x_speed = base_speed  # 使用基础直线速度
            control_mode = "直线模式"
        
        # 应用低通滤波器平滑速度输出（提高稳定性）
        self.z_speed_filtered = self.filter_alpha * z_speed + (1 - self.filter_alpha) * self.z_speed_filtered
        self.x_speed_filtered = self.filter_alpha * x_speed + (1 - self.filter_alpha) * self.x_speed_filtered
        
        # 限制最大速度变化率（防止剧烈变化）
        max_z_change = 50.0  # 最大转向速度变化率
        max_x_change = 0.3   # 最大前进速度变化率
        
        # 确保速度变化平滑
        z_speed_smooth = max(min(self.z_speed_filtered, self.last_z_speed + max_z_change), 
                            self.last_z_speed - max_z_change)
        x_speed_smooth = max(min(self.x_speed_filtered, self.last_x_speed + max_x_change), 
                            self.last_x_speed - max_x_change)
        
        # 发送平滑后的控制命令
        self.ep_chassis.drive_speed(x=x_speed_smooth, y=0, z=z_speed_smooth, timeout=0.1)
        
        # 更新状态 - 记录最后一次速度用于丢线恢复
        self.last_error = float(error)
        self.last_z_speed = float(z_speed_smooth)
        self.last_x_speed = float(x_speed_smooth)
        self.line_lost_count = 0
        
        # 显示滤波后的速度信息
        cv2.putText(frame, f"平滑X速度: {x_speed_smooth:.2f}", (10, 330), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"平滑Z速度: {z_speed_smooth:.1f}", (10, 360), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
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
    
    def _handle_line_lost(self, frame: np.ndarray) -> None:
        """
        处理未检测到线条的情况 - 增强稳定性版本
        
        参数:
            frame: 当前帧（用于调试）
        """
        self.line_lost_count += 1
        
        if self.line_lost_count > self.config.max_line_lost_frames:
            if not self.searching:
                self.searching = True
                self.search_start_time = time.time()
                print(f"🚨 线条丢失，开始智能找回 - 最后方向: {self.last_curve_direction}")
            
            elapsed_time = time.time() - self.search_start_time
            height, width = frame.shape[:2]
            
            # 超时处理：如果搜索时间过长，采取更积极的找回策略
            if elapsed_time > self.config.search_timeout_seconds:
                # 超时后的紧急找回策略
                if self.last_curve_direction == "left":
                    # 左转超时：大幅度左转 + 减速前进
                    z_speed = -180  # 固定左转速度
                    x_speed = 0.3   # 慢速前进
                    search_status = "紧急左转找回"
                    recovery_reason = "左转超时"
                elif self.last_curve_direction == "right":
                    # 右转超时：大幅度右转 + 减速前进
                    z_speed = 180   # 固定右转速度
                    x_speed = 0.3   # 慢速前进
                    search_status = "紧急右转找回"
                    recovery_reason = "右转超时"
                else:
                    # 直线超时：小幅左右摆动 + 慢速前进
                    z_speed = 100 * np.sin(time.time() * 2)  # 正弦摆动
                    x_speed = 0.4
                    search_status = "摆动搜索"
                    recovery_reason = "直线超时"
                
                print(f"⚠️ 搜索超时，启动紧急找回: {recovery_reason}")
            else:
                # 正常搜索阶段的智能找回策略
                if self.last_curve_direction != "straight":
                    # 急弯冲出去：根据最后方向智能找回
                    if self.last_curve_direction == "left":
                        # 左转冲出去：温和左转 + 适度前进
                        z_speed = max(-250, self.last_z_speed * 1.4)  # 1.4倍转向力度
                        x_speed = min(0.6, self.last_x_speed * 1.05)  # 1.05倍前进速度
                        search_status = "智能左转找回"
                        recovery_reason = "左急弯冲出"
                    else:  # right
                        # 右转冲出去：温和右转 + 适度前进
                        z_speed = min(250, self.last_z_speed * 1.4)   # 1.4倍转向力度
                        x_speed = min(0.6, self.last_x_speed * 1.05)  # 1.05倍前进速度
                        search_status = "智能右转找回"
                        recovery_reason = "右急弯冲出"
                else:
                    # 直线冲出去：保持速度继续前进 + 小幅摆动
                    z_speed = 30 * np.sin(time.time() * 1.5)  # 小幅正弦摆动
                    x_speed = min(0.8, self.last_x_speed * 1.05)  # 轻微加速
                    search_status = "保持前进+摆动"
                    recovery_reason = "直线冲出"
            
            # 应用速度平滑滤波
            z_speed_smooth = self.filter_alpha * z_speed + (1 - self.filter_alpha) * self.z_speed_filtered
            x_speed_smooth = self.filter_alpha * x_speed + (1 - self.filter_alpha) * self.x_speed_filtered
            
            # 执行智能找回策略
            try:
                self.ep_chassis.drive_speed(x=x_speed_smooth, y=0, z=z_speed_smooth, timeout=0.1)
            except Exception as e:
                print(f"⚠️ 控制命令发送失败: {e}")
                # 失败后尝试重新初始化连接
                try:
                    self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
                except:
                    pass
            
            # 更新滤波状态
            self.z_speed_filtered = z_speed_smooth
            self.x_speed_filtered = x_speed_smooth
            
            # 增强调试信息显示
            cv2.putText(frame, f"🔍 {search_status}", 
                       (width // 2 - 120, height // 2 - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, f"📊 原因: {recovery_reason}",
                       (width // 2 - 120, height // 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(frame, f"🎯 最后方向: {self.last_curve_direction}",
                       (width // 2 - 120, height // 2 + 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(frame, f"⚡ 速度: X={x_speed_smooth:.2f}m/s Z={z_speed_smooth:.1f}°/s",
                       (width // 2 - 120, height // 2 + 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(frame, f"⏱️ 已丢失: {elapsed_time:.1f}秒",
                       (width // 2 - 120, height // 2 + 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
            cv2.putText(frame, f"📈 丢线计数: {self.line_lost_count}",
                       (width // 2 - 120, height // 2 + 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
        else:
            # 短暂丢线，保持最后的速度继续前进
            try:
                self.ep_chassis.drive_speed(x=self.last_x_speed, y=0, z=self.last_z_speed, timeout=0.1)
            except Exception as e:
                print(f"⚠️ 短暂丢线控制失败: {e}")
    
    
    def run(self) -> None:
        """巡线机器人的主执行循环"""
        try:
            while True:
                frame = self.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
                if frame is None:
                    continue
                
                line_centers, line_rows, future_curvature = self._process_frame(frame)
                
                if line_centers:
                    self._handle_line_detected(line_centers, line_rows, future_curvature, frame)
                else:
                    self._handle_line_lost(frame)
                
                # 显示调试窗口
                cv2.imshow("图像帧", frame)
                cv2.imshow("掩膜", cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()
    
    def cleanup(self) -> None:
        """清理资源并停止机器人"""
        try:
            self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
            time.sleep(0.5)
            self.ep_camera.stop_video_stream()
            self.ep_robot.close()
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"清理过程中出错: {e}")
