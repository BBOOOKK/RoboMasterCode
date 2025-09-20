# RoboMaster S1 巡线机器人调试手册

## 配置文件参数详解

### 连接设置
- `connection_type`: 连接模式 ("ap" - 直连模式, "sta" - 路由器模式)
- **调试建议**: 如果连接失败，检查机器人是否开机，网络设置是否正确

### 颜色检测设置
- `lower_blue`: 蓝色HSV下限阈值 (H, S, V)
- `upper_blue`: 蓝色HSV上限阈值 (H, S, V)
- **调试建议**: 如果检测不到蓝线，调整HSV范围以适应环境光线

### ROI区域设置
- `roi_row_offsets`: 检测区域行偏移量 (从远到近)
- `min_white_pixels`: 最小白色像素数阈值
- **调试建议**: 增加偏移量可扩大检测范围，减少则聚焦近处

### 未来线路预测
- `prediction_distance`: 预测未来像素距离
- `early_curve_threshold`: 提前检测弯道的曲率阈值
- `preemptive_slowdown_factor`: 提前降速系数
- **调试建议**: 提高阈值可减少误报，降低阈值可更早检测弯道

### PID控制器设置
- `main_pid_params`: 主PID参数 (P, I, D, 输出限制)
- `curved_pid_params`: 弯道PID参数 (P, I, D, 输出限制)
- **调试建议**: P值影响响应速度，I值消除稳态误差，D值抑制超调

### 云台设置
- `gimbal_pitch`: 云台俯仰角度
- `gimbal_yaw`: 云台偏航角度
- `gimbal_speed`: 云台移动速度
- **调试建议**: 调整俯仰角度可改变摄像头视野范围

### 丢线恢复设置
- `max_line_lost_frames`: 最大丢线帧数
- `search_timeout_seconds`: 搜索超时时间
- `search_speed_degrees`: 搜索转向速度
- **调试建议**: 增加超时时间给机器人更多恢复机会

### 历史数据设置
- `position_history_length`: 位置历史数据点数
- `curvature_history_length`: 曲率历史数据点数
- **调试建议**: 增加历史点数可平滑数据但降低响应速度

### 速度设置
- `base_speed_straight`: 直线基础速度
- `base_speed_curve`: 弯道基础速度
- **调试建议**: 根据赛道复杂度调整速度

### 曲率阈值设置
- `curvature_enter_threshold`: 进入弯道阈值
- `curvature_exit_threshold`: 退出弯道阈值
- **调试建议**: 降低阈值可更敏感检测弯道

---

## 竞速激进方案 (1-3级)

### 级别1: 入门竞速
```python
# PID 控制器设置
main_pid_params = (0.6, 0.0008, 0.18, 150)     # 响应较快
curved_pid_params = (2.8, 0.0012, 0.5, 280)    # 弯道控制较强

# 速度设置  
base_speed_straight = 1.2         # 中等速度
base_speed_curve = 0.4            # 弯道适当减速

# 曲率阈值设置
curvature_enter_threshold = 0.07   # 较早检测弯道
curvature_exit_threshold = 0.04    # 较快退出弯道模式
```

### 级别2: 中级竞速
```python
# PID 控制器设置
main_pid_params = (0.7, 0.0010, 0.20, 170)     # 更强响应
curved_pid_params = (3.0, 0.0015, 0.55, 300)   # 极强弯道控制

# 速度设置
base_speed_straight = 1.4         # 较高速度
base_speed_curve = 0.5            # 弯道保持较高速度

# 曲率阈值设置
curvature_enter_threshold = 0.06   # 更早检测弯道
curvature_exit_threshold = 0.03    # 更快退出弯道模式
```

### 级别3: 专业竞速
```python
# PID 控制器设置
main_pid_params = (0.8, 0.0012, 0.22, 190)     # 超强响应
curved_pid_params = (3.2, 0.0018, 0.60, 320)   # 极限弯道控制

# 速度设置
base_speed_straight = 1.6         # 极限速度
base_speed_curve = 0.6            # 弯道几乎不减速

# 曲率阈值设置
curvature_enter_threshold = 0.05   # 最早检测弯道
curvature_exit_threshold = 0.02    # 立即退出弯道模式
```

---

## 经典稳定方案 (1-3级)

### 级别1: 基础稳定
```python
# PID 控制器设置
main_pid_params = (0.4, 0.0004, 0.12, 110)     # 温和响应
curved_pid_params = (2.2, 0.0008, 0.40, 240)   # 稳定弯道控制

# 速度设置
base_speed_straight = 0.8         # 安全速度
base_speed_curve = 0.2            # 弯道显著减速

# 曲率阈值设置
curvature_enter_threshold = 0.10   # 较晚检测弯道
curvature_exit_threshold = 0.07    # 较晚退出弯道模式
```

### 级别2: 标准稳定
```python
# PID 控制器设置
main_pid_params = (0.45, 0.0005, 0.14, 120)    # 标准响应
curved_pid_params = (2.4, 0.0009, 0.42, 250)   # 标准弯道控制

# 速度设置
base_speed_straight = 1.0         # 标准速度
base_speed_curve = 0.25           # 标准弯道减速

# 曲率阈值设置
curvature_enter_threshold = 0.09   # 标准检测时机
curvature_exit_threshold = 0.06    # 标准退出时机
```

### 级别3: 高级稳定
```python
# PID 控制器设置
main_pid_params = (0.5, 0.0006, 0.16, 130)     # 精确响应
curved_pid_params = (2.6, 0.0010, 0.45, 260)   # 精确弯道控制

# 速度设置
base_speed_straight = 1.1         # 较快但稳定
base_speed_curve = 0.3            # 适度弯道减速

# 曲率阈值设置
curvature_enter_threshold = 0.08   # 较早检测弯道
curvature_exit_threshold = 0.05    # 较早退出弯道模式
```

---

## 常见问题调试指南

### 1. 检测不到蓝线
- **检查**: HSV颜色范围是否匹配环境光线
- **调整**: 使用 `cv2.imshow("掩膜", mask)` 查看掩膜效果
- **建议**: 在不同光线条件下重新校准HSV值

### 2. 弯道转向不足或过度
- **检查**: 弯道PID参数和曲率阈值
- **调整**: 逐步调整P值和曲率阈值
- **建议**: 从稳定方案开始调试，逐步向竞速方案过渡

### 3. 直线行驶不稳定
- **检查**: 主PID参数和历史数据长度
- **调整**: 增加I值减少稳态误差，调整历史数据平滑度
- **建议**: 确保ROI检测区域设置合理

### 4. 丢线后无法恢复
- **检查**: 搜索超时时间和转向速度
- **调整**: 增加超时时间，调整搜索方向策略
- **建议**: 基于最后一次弯道方向优化搜索策略

### 5. 速度控制不理想
- **检查**: 基础速度设置和弯道减速方案
- **调整**: 根据赛道复杂度分级设置速度
- **建议**: 在直道和弯道分别测试速度响应

---

## 调试工具使用

1. **实时显示**: 启用 `cv2.imshow` 查看图像处理和检测结果
2. **参数打印**: 在控制台查看误差、速度、曲率等实时数据
3. **性能分析**: 使用Python性能分析工具优化关键函数
4. **日志记录**: 添加日志记录功能追踪机器人状态变化

通过系统化的参数调整和问题排查，可以优化机器人的巡线性能，适应不同的赛道环境和竞赛要求。
