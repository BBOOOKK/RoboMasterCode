# RoboMaster S1 巡线机器人调试指南

## 🎯 核心调试功能

### 实时视觉调试窗口
运行程序后会自动打开两个调试窗口：
- **图像帧窗口**: 显示原始图像+检测结果可视化
- **掩膜窗口**: 显示蓝色掩膜处理结果

### 关键调试信息显示
在图像左上角实时显示：
- **误差**: 当前中心位置与图像中心的偏差（像素）
- **X速度**: 前进速度（m/s）
- **Z速度**: 旋转速度（度/秒）  
- **曲率**: 当前检测到的线条曲率
- **预测曲率**: 前方50像素处的预测曲率
- **模式**: 当前控制模式（常规/漂移）
- **方向**: 弯道方向（左/右/直道）

## 🔧 参数调试方法

### 使用参数管理工具
```bash
# 列出所有可调参数
python manage_params.py list

# 获取特定参数值
python manage_params.py get base_speed_straight
python manage_params.py get drift_enabled

# 设置参数值（自动类型转换）
python manage_params.py set base_speed_straight 1.2
python manage_params.py set drift_aggression 0.9
python manage_params.py set inner_wheel_reduction 0.65

# 重置参数到默认值
python manage_params.py reset base_speed_straight
python manage_params.py reset  # 重置所有参数
```

### 关键调试参数列表

#### 速度控制参数
- `base_speed_straight`: 直线基础速度 (0.5-2.0)
- `base_speed_curve`: 弯道基础速度 (0.5-2.0) 
- `drift_speed_boost`: 漂移速度加成 (1.0-1.5)

#### 漂移控制参数
- `drift_enabled`: 是否启用漂移模式 (true/false)
- `min_drift_curvature`: 开始漂移的最小曲率 (0.1-0.4)
- `inner_wheel_reduction`: 内侧轮减速系数 (0.6-0.8)
- `outer_wheel_boost`: 外侧轮加速系数 (0.2-0.4)
- `drift_aggression`: 漂移激进程度 (0.0-1.0)

#### 视觉检测参数
- `lower_blue`: 蓝色检测下限 (HSV格式)
- `upper_blue`: 蓝色检测上限 (HSV格式) 
- `roi_row_offsets`: ROI检测行偏移 [150,120,90,60,30]
- `min_white_pixels`: 最小白色像素数 (5-15)

#### PID控制参数
- `main_pid_params`: 主PID参数 [P,I,D,输出限制]
- `curved_pid_params`: 弯道PID参数 [P,I,D,输出限制]

#### 预测参数
- `prediction_distance`: 预测距离像素 (30-70)
- `early_curve_threshold`: 提前检测阈值 (0.1-0.2)
- `preemptive_slowdown_factor`: 提前降速系数 (0.6-0.9)

## 🚀 性能优化调试流程

### 第一步：基础巡线调试
1. 确保蓝色检测范围正确（调整`lower_blue`/`upper_blue`）
2. 调试直线巡线稳定性（调整`main_pid_params`）
3. 优化基础速度（设置`base_speed_straight`）

### 第二步：弯道性能优化  
1. 启用漂移模式：`set drift_enabled true`
2. 调整漂移激进程度：`set drift_aggression 0.8`
3. 优化轮速差参数：`inner_wheel_reduction`和`outer_wheel_boost`
4. 设置漂移触发阈值：`min_drift_curvature`

### 第三步：预测系统调试
1. 观察"预测曲率"显示值
2. 调整预测距离：`prediction_distance`
3. 设置提前降速系数：`preemptive_slowdown_factor`
4. 优化提前检测阈值：`early_curve_threshold`

## ⚠️ 常见问题调试

### 检测不到蓝线
- 检查环境光照条件
- 调整`lower_blue`和`upper_blue`参数
- 增加`min_white_pixels`值

### 过弯时冲出赛道
- 降低`base_speed_curve`
- 增加`inner_wheel_reduction` 
- 减小`drift_aggression`
- 提前触发降速：降低`early_curve_threshold`

### 漂移过度或不足
- 调整`inner_wheel_reduction`（0.6-0.8）
- 调整`outer_wheel_boost`（0.2-0.4）
- 修改`min_drift_curvature`触发阈值

### 直线行驶不稳定
- 优化`main_pid_params`的P值
- 检查ROI检测是否稳定
- 调整`position_history_length`

## 🎮 实时调试技巧

1. **观察曲率值**: 曲率>0.25时进入漂移模式
2. **关注预测信息**: 预测曲率>0.15时提前降速
3. **监控速度变化**: X速度保持前进，Z速度控制转向
4. **注意模式切换**: 在"常规模式"和"漂移中"之间切换

## 📊 调试参数推荐值

### 激进竞速模式
```
base_speed_straight: 1.8
base_speed_curve: 1.6
drift_aggression: 0.9
inner_wheel_reduction: 0.65
outer_wheel_boost: 0.35
```

### 稳定巡线模式  
```
base_speed_straight: 1.2
base_speed_curve: 1.0
drift_aggression: 0.7
inner_wheel_reduction: 0.75
outer_wheel_boost: 0.25
```

### 新手调试模式
```
base_speed_straight: 0.8
base_speed_curve: 0.6  
drift_enabled: false
```

## 🔚 调试完成检查

✅ 蓝色检测稳定准确
✅ 直线巡线平滑无振荡  
✅ 弯道过渡自然流畅
✅ 漂移控制精准有力
✅ 预测系统提前预警
✅ 参数响应及时有效

使用`manage_params.py export my_config.yaml`导出优化后的配置！
