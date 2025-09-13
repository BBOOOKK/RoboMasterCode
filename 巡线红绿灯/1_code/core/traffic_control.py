
"""
test_traffic_control.py
-----------------------
交通信号灯（红停、绿行）识别模块 —— 可被工程内其他脚本/模块调用。

设计目标：
1) 不改动你现有项目逻辑与参数；本模块仅做“颜色判定”，不发底盘速度指令；
2) 保留你在 Robomaster.py 中的阈值、ROI 与形态学处理思路；
3) 提供清晰的 API：TrafficLightDetector.detect(frame) -> (color, info)；
4) 需要可视化时，提供 draw_debug(frame, info, label) 进行叠画；
5) 作为脚本独立运行时，提供一个最小演示（不会在被 import 时执行）。
"""

from dataclasses import dataclass
from typing import Tuple, Optional, Dict
import numpy as np
import cv2

# ===============================
# 配置与数据结构
# ===============================

@dataclass
class TLConfig:
    """交通灯识别配置（与 Robomaster.py 保持一致的默认值）"""
    # ROI：y1, x1, y2, x2（按比例）
    roi_box: Tuple[float, float, float, float] = (0.15, 0.35, 0.70, 0.65)
    # HSV 阈值（BGR->HSV 后）
    red1: Tuple[Tuple[int,int,int], Tuple[int,int,int]] = ((0, 80, 80), (10, 255, 255))
    red2: Tuple[Tuple[int,int,int], Tuple[int,int,int]] = ((170, 80, 80), (180, 255, 255))
    green: Tuple[Tuple[int,int,int], Tuple[int,int,int]] = ((40, 70, 70), (85, 255, 255))
    # 形态学与判定
    kernel_size: int = 5
    area_ratio: float = 0.003   # 面积相对阈值（ROI 的 0.3%）
    dominance: float = 1.2      # 主导比例：A > B * 1.2 则判 A
    # 圆形检测参数 - 提高要求避免误识别
    min_circularity: float = 0.85  # 最小圆形度（0-1，越接近1越圆）提高要求
    min_radius: int = 15         # 最小半径（像素）提高要求
    max_radius: int = 45         # 最大半径（像素）缩小范围
    # 额外形状检测参数
    aspect_ratio_max: float = 1.3  # 最大宽高比（避免识别长条形红色物体）

# ===============================
# 核心检测器
# ===============================

class TrafficLightDetector:
    """交通灯颜色识别器（纯视觉，不控制底盘）"""
    def __init__(self, cfg: Optional[TLConfig] = None):
        self.cfg = cfg or TLConfig()
        k = self.cfg.kernel_size
        self.kernel = np.ones((k, k), np.uint8)

    @staticmethod
    def _max_area(mask) -> Tuple[int, Optional[np.ndarray]]:
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return 0, None
        c = max(cnts, key=cv2.contourArea)
        return int(cv2.contourArea(c)), c

    @staticmethod
    def _is_circular(contour, min_circularity=0.85, min_radius=15, max_radius=45, max_aspect_ratio=1.3) -> bool:
        """
        检测轮廓是否为圆形
        
        参数:
            contour: 轮廓
            min_circularity: 最小圆形度 (0-1)
            min_radius: 最小半径
            max_radius: 最大半径
            max_aspect_ratio: 最大宽高比
            
        返回:
            bool: 是否为圆形
        """
        if contour is None:
            return False
            
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        # 避免除零错误
        if perimeter <= 0:
            return False
            
        # 计算圆形度: 4π * area / perimeter²
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        
        # 计算等效半径
        equivalent_radius = np.sqrt(area / np.pi)
        
        # 计算宽高比（避免识别长条形物体）
        rect = cv2.minAreaRect(contour)
        width, height = rect[1]
        if width > 0 and height > 0:
            aspect_ratio = max(width, height) / min(width, height)
        else:
            aspect_ratio = 1.0
        
        # 检查圆形度、半径范围和宽高比
        return (circularity >= min_circularity and 
                min_radius <= equivalent_radius <= max_radius and
                aspect_ratio <= max_aspect_ratio)

    def detect(self, bgr: np.ndarray) -> Tuple[str, Dict[str, object]]:
        """
        输入：BGR 图像帧（np.ndarray）
        输出：
            color: "red" | "green" | "none"
            info:  调试信息字典（roi, red_area, green_area, red_cnt, grn_cnt）
        说明：仅做颜色判定，不发速度与 LED 指令；调用者按 color 决策。
        """
        h, w = bgr.shape[:2]
        y1 = int(h * self.cfg.roi_box[0]); x1 = int(w * self.cfg.roi_box[1])
        y2 = int(h * self.cfg.roi_box[2]); x2 = int(w * self.cfg.roi_box[3])
        roi = bgr[y1:y2, x1:x2]

        # 预处理
        blur = cv2.GaussianBlur(roi, (5,5), 0)
        hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # 掩膜
        r1 = cv2.inRange(hsv, np.array(self.cfg.red1[0]),  np.array(self.cfg.red1[1]))
        r2 = cv2.inRange(hsv, np.array(self.cfg.red2[0]),  np.array(self.cfg.red2[1]))
        red_mask   = cv2.morphologyEx(cv2.bitwise_or(r1, r2), cv2.MORPH_OPEN, self.kernel, iterations=1)
        green_mask = cv2.morphologyEx(
            cv2.inRange(hsv, np.array(self.cfg.green[0]), np.array(self.cfg.green[1])),
            cv2.MORPH_OPEN, self.kernel, iterations=1
        )

        # 面积阈值（相对）
        roi_area = roi.shape[0] * roi.shape[1]
        th = max(150, int(roi_area * self.cfg.area_ratio))

        red_area, red_c = self._max_area(red_mask)
        grn_area, grn_c = self._max_area(green_mask)

        color = "none"
        # 红色检测：需要满足面积条件且为圆形（增加宽高比检测）
        if red_area > th and red_area > grn_area * self.cfg.dominance:
            if self._is_circular(red_c, self.cfg.min_circularity, self.cfg.min_radius, self.cfg.max_radius, self.cfg.aspect_ratio_max):
                color = "red"
        # 绿色检测：只需要满足面积条件（绿灯可以是各种形状）
        elif grn_area > th and grn_area > red_area * self.cfg.dominance:
            color = "green"

        # 计算红色区域的圆形度（用于调试信息）
        red_circular = False
        red_circularity = 0.0
        red_radius = 0.0
        if red_c is not None:
            red_circular = self._is_circular(red_c, self.cfg.min_circularity, self.cfg.min_radius, self.cfg.max_radius)
            area = cv2.contourArea(red_c)
            perimeter = cv2.arcLength(red_c, True)
            if perimeter > 0:
                red_circularity = 4 * np.pi * area / (perimeter * perimeter)
                red_radius = np.sqrt(area / np.pi)

        info = {
            "roi": (x1, y1, x2, y2),
            "red_area": int(red_area),
            "green_area": int(grn_area),
            "red_cnt": red_c,
            "grn_cnt": grn_c,
            "red_circular": red_circular,
            "red_circularity": float(red_circularity),
            "red_radius": float(red_radius)
        }
        return color, info

# ===============================
# 可选：调试绘图工具
# ===============================

def draw_debug(frame: np.ndarray, info: Dict[str, object], label: str) -> None:
    """在帧上叠画 ROI 与文字（原地修改 frame）"""
    x1, y1, x2, y2 = info["roi"]
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
    
    # 显示圆形度信息
    if "red_circular" in info:
        circular_info = f"圆形: {'是' if info['red_circular'] else '否'} 圆形度: {info['red_circularity']:.2f} 半径: {info['red_radius']:.1f}"
        txt = f"{label}  R:{info['red_area']}  G:{info['green_area']}  {circular_info}"
    else:
        txt = f"{label}  R:{info['red_area']}  G:{info['green_area']}"
        
    cv2.putText(frame, txt, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    # 如果检测到红色轮廓且为圆形，绘制圆形
    if info.get("red_circular", False) and info["red_cnt"] is not None:
        # 计算最小外接圆
        (x, y), radius = cv2.minEnclosingCircle(info["red_cnt"])
        center = (int(x) + x1, int(y) + y1)  # 转换为全局坐标
        radius = int(radius)
        cv2.circle(frame, center, radius, (0, 0, 255), 2)
        cv2.circle(frame, center, 2, (0, 0, 255), 3)

# ===============================
# 独立演示（不会影响被 import 的场景）
# ===============================

if __name__ == "__main__":
    # 仅用于本文件单独测试：显示窗口，按 q 退出
    from robomaster import robot
    SHOW_WIN = True

    ep = robot.Robot()
    ep.initialize(conn_type="ap")
    ep.camera.start_video_stream(display=False)

    detector = TrafficLightDetector()

    try:
        while True:
            frame = ep.camera.read_cv2_image(strategy="newest", timeout=0.5)
            if frame is None:
                continue
            color, info = detector.detect(frame)
            if SHOW_WIN:
                draw_debug(frame, info, color.upper())
                cv2.imshow("traffic_light_demo", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break
    finally:
        try: ep.chassis.drive_speed(0,0,0)
        except: pass
        ep.camera.stop_video_stream()
        ep.close()
        if SHOW_WIN:
            cv2.destroyAllWindows()
