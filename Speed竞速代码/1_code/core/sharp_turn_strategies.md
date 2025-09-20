# 大角度及直角弯应对策略方案

## 方案一：多级曲率检测 + 动态PID切换

### 核心思想
根据曲率大小分级处理，不同级别的弯道使用不同的控制策略

### 实现方案
```python
# 在配置中添加多级曲率阈值
sharp_turn_threshold: float = 0.15      # 急弯阈值
right_angle_threshold: float = 0.25     # 直角弯阈值

# 添加对应的PID参数
sharp_turn_pid_params: Tuple[float, float, float, float] = (4.0, 0.002, 0.7, 350)
right_angle_pid_params: Tuple[float, float, float, float] = (5.0, 0.003, 0.9, 400)

# 在_handle_line_detected中修改控制逻辑
if curvature > self.config.right_angle_threshold:
    # 直角弯模式
    z_speed = self.right_angle_pid.update(0, error)
    x_speed = self.config.base_speed_curve * 0.3  # 大幅减速
elif curvature > self.config.sharp_turn_threshold:
    # 急弯模式
    z_speed = self.sharp_turn_pid.update(0, error)
    x_speed = self.config.base_speed_curve * 0.5  # 中等减速
elif use_curve_pid:
    # 普通弯道模式
    z_speed = self.curved_pid.update(0, error)
    x_speed = self._calculate_curve_speed(curvature, self.config.base_speed_curve)
```

### 优点
- 分级处理，针对性更强
- 参数可独立调整
- 适应不同类型的弯道

### 缺点
- 需要调试的参数增多
- 状态切换可能产生震荡

---

## 方案二：前瞻性预测 + 预转向策略

### 核心思想
利用前方检测点预测弯道趋势，提前进行转向

### 实现方案
```python
def _predict_turn_intensity(self, line_centers: List[int], line_rows: List[int]) -> float:
    """预测弯道急缓程度"""
    if len(line_centers) < 4:
        return 0.0
    
    # 计算前方点的曲率变化率
    front_points = sorted(zip(line_rows, line_centers))[:4]
    front_rows = [p[0] for p in front_points]
    front_centers = [p[1] for p in front_points]
    
    # 计算曲率变化梯度
    curvature_gradient = self._calculate_curvature_gradient(front_centers, front_rows)
    return abs(curvature_gradient)

def _calculate_curvature_gradient(self, centers: List[int], rows: List[int]) -> float:
    """计算曲率变化梯度"""
    if len(centers) < 3:
        return 0.0
    
    # 使用差分法计算曲率变化率
    curvatures = []
    for i in range(len(centers) - 2):
        sub_centers = centers[i:i+3]
        sub_rows = rows[i:i+3]
        curvatures.append(self.calculate_curvature(sub_centers, sub_rows))
    
    if len(curvatures) < 2:
        return 0.0
    
    return curvatures[-1] - curvatures[0]

# 在控制逻辑中添加预转向
turn_intensity = self._predict_turn_intensity(line_centers, line_rows)
if turn_intensity > 0.1:  # 弯道变急
    # 增加转向力度预判
    additional_turn = turn_intensity * 50  # 根据急缓程度增加转向
    z_speed += additional_turn
```

### 优点
- 提前预判，响应更快
- 平滑过渡，减少过冲
- 自适应不同弯道形状

### 缺点
- 算法复杂度较高
- 需要精确的预测模型

---

## 方案三：轨迹跟踪 + 最优路径规划

### 核心思想
基于检测到的点进行轨迹拟合，规划最优过弯路径

### 实现方案
```python
def _plan_optimal_path(self, line_centers: List[int], line_rows: List[int]) -> Tuple[float, float]:
    """规划最优过弯路径"""
    if len(line_centers) < 5:
        return 0, self.config.base_speed_straight
    
    # 多项式拟合轨迹
    try:
        coefficients = np.polyfit(line_rows, line_centers, 3)  # 三次多项式拟合
        a, b, c, d = coefficients
        
        # 计算最优转向点（曲率最大点）
        # 对于三次函数，求二阶导数为0的点
        optimal_y = -b / (3 * a) if a != 0 else line_rows[0]
        
        # 预测未来位置和所需转向
        future_x = a * optimal_y**3 + b * optimal_y**2 + c * optimal_y + d
        error = self.frame_center - future_x
        
        # 根据弯道急缓调整速度
        max_curvature = self._calculate_max_curvature(coefficients, line_rows)
        speed_factor = 1.0 / (1.0 + max_curvature * 3.0)  # 曲率越大，减速越多
        
        return error, self.config.base_speed_straight * speed_factor
        
    except:
        return 0, self.config.base_speed_straight

# 在主要控制逻辑中使用
optimal_error, optimal_speed = self._plan_optimal_path(line_centers, line_rows)
z_speed = self.curved_pid.update(0, optimal_error)
x_speed = optimal_speed
```

### 优点
- 数学上最优的过弯路径
- 平滑的速度控制
- 适应各种复杂弯道

### 缺点
- 计算量较大
- 需要较高的数学建模能力

---

## 方案四：机器学习自适应策略

### 核心思想
使用简单的机器学习方法自适应调整参数

### 实现方案
```python
class AdaptiveTurnController:
    def __init__(self):
        self.learning_rate = 0.1
        self.turn_parameters = {
            'p_gain': 3.0,
            'speed_reduction': 0.5
        }
        self.previous_performance = 0
    
    def update_parameters(self, current_error: float, previous_error: float) -> None:
        """根据性能反馈调整参数"""
        performance_improvement = abs(previous_error) - abs(current_error)
        
        if performance_improvement > 0:
            # 性能提升，保持或微调
            self.turn_parameters['p_gain'] *= 1.05
        else:
            # 性能下降，调整参数
            self.turn_parameters['p_gain'] *= 0.95
            self.turn_parameters['speed_reduction'] += 0.05
        
        # 参数边界限制
        self.turn_parameters['p_gain'] = max(1.0, min(6.0, self.turn_parameters['p_gain']))
        self.turn_parameters['speed_reduction'] = max(0.2, min(0.8, self.turn_parameters['speed_reduction']))

# 在主类中使用
self.adaptive_controller = AdaptiveTurnController()

# 在每次控制循环中
self.adaptive_controller.update_parameters(error, self.last_error)
z_speed = self.turn_parameters['p_gain'] * error
x_speed = self.config.base_speed_straight * (1 - self.turn_parameters['speed_reduction'])
```

### 优点
- 自适应环境变化
- 减少手动调参工作
- 持续优化性能

### 缺点
- 需要时间学习收敛
- 可能产生不稳定的学习过程

---

## 方案五：混合策略（推荐）

### 核心思想
结合多种策略的优点，根据实际情况选择最佳方案

### 实现方案
```python
def _handle_sharp_turns(self, line_centers: List[int], line_rows: List[int], 
                       curvature: float, error: float) -> Tuple[float, float]:
    """处理急弯和直角弯的混合策略"""
    
    # 策略1：多级曲率检测（基础）
    if curvature > 0.25:
        # 直角弯：强力转向 + 大幅减速
        return self.right_angle_pid.update(0, error), self.config.base_speed_curve * 0.3
    elif curvature > 0.15:
        # 急弯：较强转向 + 中等减速
        return self.sharp_turn_pid.update(0, error), self.config.base_speed_curve * 0.5
    
    # 策略2：前瞻性预测（增强）
    turn_intensity = self._predict_turn_intensity(line_centers, line_rows)
    if turn_intensity > 0.1:
        base_z_speed = self.curved_pid.update(0, error)
        additional_turn = turn_intensity * 40
        return base_z_speed + additional_turn, self.config.base_speed_curve * 0.7
    
    # 策略3：轨迹规划（优化）
    optimal_error, optimal_speed = self._plan_optimal_path(line_centers, line_rows)
    return self.curved_pid.update(0, optimal_error), optimal_speed
    
    # 默认策略
    return self.curved_pid.update(0, error), self._calculate_curve_speed(curvature, self.config.base_speed_curve)
```

### 推荐理由
- 综合各种策略优点
- 根据弯道特性自动选择最佳方案
- 既有基础保障又有优化提升
- 易于调试和扩展

## 实施建议

1. **从方案一开始**：最容易实现，效果明显
2. **逐步添加方案二**：提升响应速度
3. **最终采用方案五**：获得最佳性能
4. **根据实际赛道调试**：不同赛道可能需要不同的参数组合

每个方案都可以独立实现，也可以组合使用，建议根据实际需求和开发资源选择合适的方案。
