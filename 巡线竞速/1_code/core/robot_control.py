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
    prediction_distance: int = 100  # 预测未来像素距离
    early_curve_threshold: float = 0.15  # 提前检测弯道的曲率阈值
    preemptive_slowdown_factor: float = 0.7  # 提前降速系数
    
    # PID 控制器设置 - 超激进弯道控制
    main_pid_params: Tuple[float, float, float, float] = (0.5, 0.0006, 0.15, 130)     # 进一步提高响应
    curved_pid_params: Tuple[float, float, float, float] = (2.5, 0.001, 0.45, 265)    # 极强弯道控制，防止转出
    
    # 云台设置
    gimbal_pitch: float = -25.0
    gimbal_yaw: float = 0.0
    gimbal_speed: int = 100
    
    # 丢线恢复设置
    max_line_lost_frames: int = 10
    search_timeout_seconds: float = 8.0
    search_speed_degrees: float = 20.0
    
    # 历史数据设置
    position_history_length: int = 8
    curvature_history_length: int = 5
    
    # 速度设置 - 微调协调性
    base_speed_straight: float = 1.0         # 微调直线速度，提升协调性
    base_speed_curve: float = 0.3            # 微调弯道速度，提升过弯协调性
    
    # 曲率阈值设置 - 大幅降低阈值，更敏感检测弯道
    curvature_enter_threshold: float = 0.08   # 大幅降低阈值，极早检测弯道
    curvature_exit_threshold: float = 0.05    # 大幅降低退出阈值，保持弯道模式
    
    # 历史数据设置 - 减少平滑，提高响应速度
    position_history_length: int = 5          # 减少历史数据点数
    curvature_history_length: int = 3         # 减少曲率历史点数


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
        self.last_z_speed: float = 0.0
        self.last_x_speed: float = 0.0
        self.last_curve_direction: str = "straight"  # 记录最后一次弯道方向
    
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
    
    def _process_frame(self, frame: np.ndarray) -> Tuple[Optional[List[int]], Optional[List[int]], Optional[float]]:
        """
        处理单帧图像以检测蓝线，包含未来线路预测
        
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
    
    def _predict_future_position(self, centers: List[int], rows: List[int], distance: int) -> Optional[float]:
        """
        预测未来线路位置
        
        参数:
            centers: 中心点列表
            rows: 对应的行坐标
            distance: 预测距离
            
        返回:
            预测的未来x坐标
        """
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
    
    def _calculate_curve_speed(self, curvature: float, base_speed: float) -> float:
        """
        计算弯道减速方案
        
        参数:
            curvature: 当前曲率
            base_speed: 基础速度
            
        返回:
            调整后的速度
        """
        # 根据曲率进行线性减速
        # 曲率越大，减速越多
        speed_reduction = min(0.8, curvature * 2.0)  # 最大减速80%
        return base_speed * (1.0 - speed_reduction)
    
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
    
    def _handle_line_lost(self, frame: np.ndarray) -> None:
        """
        处理未检测到线条的情况 - 增强恢复策略
        
        参数:
            frame: 当前帧（用于调试）
        """
        self.line_lost_count += 1
        
        if self.line_lost_count > self.config.max_line_lost_frames:
            if not self.searching:
                self.searching = True
                self.search_start_time = time.time()
                print("线条丢失，开始增强寻找...")
            
            elapsed_time = time.time() - self.search_start_time
            height, width = frame.shape[:2]
            
            if elapsed_time > self.config.search_timeout_seconds:
                # 超时 - 改变方向并减速
                self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
                self.search_direction *= -1
                self.search_start_time = time.time()
                
                cv2.putText(frame, "搜索超时 - 改变方向",
                           (width // 2 - 150, height // 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                # 增强搜索策略：基于最后一次弯道方向加大力度转向
                if self.last_curve_direction != "straight":
                    # 根据最后一次弯道方向进行强力转向
                    if self.last_curve_direction == "left":
                        search_speed = self.config.search_speed_degrees * 1.8  # 左转加强
                    else:  # right
                        search_speed = self.config.search_speed_degrees * -1.8  # 右转加强
                    search_status = f"强力{self.last_curve_direction}转寻找"
                else:
                    # 常规搜索
                    search_speed = self.config.search_speed_degrees * self.search_direction
                    search_status = "常规搜索"
                
                # 保持轻微前进动力，帮助恢复
                self.ep_chassis.drive_speed(x=0.2, y=0, z=search_speed, timeout=0.1)
                self.last_z_speed = float(search_speed)
                self.last_x_speed = 0.2
                
                cv2.putText(frame, f"{search_status}... {int(self.config.search_timeout_seconds - elapsed_time)}秒",
                           (width // 2 - 100, height // 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(frame, f"最后方向: {self.last_curve_direction}",
                           (width // 2 - 100, height // 2 + 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    def _emergency_stop(self) -> None:
        """紧急停止和安全恢复"""
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
