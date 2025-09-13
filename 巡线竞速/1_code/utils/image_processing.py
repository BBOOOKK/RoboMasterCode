"""
RoboMaster 巡线图像处理工具函数
包含可重用的图像去噪、颜色检测和处理函数
"""

import cv2
import numpy as np
from typing import Tuple, Optional


def denoise_image(image: np.ndarray, kernel_size: int = 5) -> np.ndarray:
    """
    对图像应用去噪操作
    
    参数:
        image: 要去噪的输入图像
        kernel_size: 形态学核的大小
        
    返回:
        去噪后的图像
    """
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    
    # 应用形态学操作
    denoised = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)  # 填充小孔
    denoised = cv2.morphologyEx(denoised, cv2.MORPH_OPEN, kernel)  # 去除小噪声点
    
    # 应用高斯模糊
    denoised = cv2.GaussianBlur(denoised, (kernel_size, kernel_size), 0)
    
    return denoised


def create_color_mask(image: np.ndarray, 
                     lower_bound: Tuple[int, int, int], 
                     upper_bound: Tuple[int, int, int]) -> np.ndarray:
    """
    为指定的HSV范围创建颜色掩膜
    
    参数:
        image: 输入的BGR图像
        lower_bound: 下限HSV边界 (H, S, V)
        upper_bound: 上限HSV边界 (H, S, V)
        
    返回:
        二进制掩膜，白色像素表示颜色在范围内
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower_bound), np.array(upper_bound))
    return mask


def detect_line_centers(mask: np.ndarray, 
                       row_positions: list, 
                       min_pixels: int = 10) -> Tuple[list, list]:
    """
    在二进制掩膜的指定行中检测线条中心
    
    参数:
        mask: 二进制掩膜图像
        row_positions: 要分析的行索引列表
        min_pixels: 考虑有效检测的最小白色像素数
        
    返回:
        (line_centers, valid_rows) 元组，其中valid_rows对应line_centers
    """
    line_centers = []
    valid_rows = []
    height, width = mask.shape
    
    for row in row_positions:
        # 确保行在边界内
        row = max(0, min(height - 1, row))
        
        # 提取行并查找白色像素
        row_pixels = mask[row, :]
        white_pixels = np.where(row_pixels == 255)[0]
        
        if len(white_pixels) > min_pixels:
            center_x = int(np.mean(white_pixels))
            line_centers.append(center_x)
            valid_rows.append(row)
    
    return line_centers, valid_rows


def draw_debug_info(image: np.ndarray, 
                   line_centers: list, 
                   valid_rows: list, 
                   frame_center: int) -> np.ndarray:
    """
    在图像上绘制调试信息
    
    参数:
        image: 要绘制的输入图像
        line_centers: 检测到的线条中心x位置列表
        valid_rows: 对应的行y位置列表
        frame_center: 帧的中心x位置
        
    返回:
        绘制了调试信息的图像
    """
    debug_image = image.copy()
    height, width = debug_image.shape[:2]
    
    # 绘制参考线和中心点
    for center_x, row in zip(line_centers, valid_rows):
        cv2.line(debug_image, (0, row), (width, row), (0, 255, 0), 1)
        cv2.circle(debug_image, (center_x, row), 5, (0, 0, 255), -1)
    
    # 绘制整体中心点
    if line_centers:
        avg_center = int(np.mean(line_centers))
        cv2.circle(debug_image, (avg_center, height - 50), 8, (255, 0, 0), -1)
    
    cv2.circle(debug_image, (frame_center, height - 50), 8, (0, 255, 255), -1)
    
    return debug_image


def calculate_frame_center(image: np.ndarray) -> int:
    """
    计算帧的中心x坐标
    
    参数:
        image: 输入图像
        
    返回:
        中心x坐标
    """
    height, width = image.shape[:2]
    return width // 2
