# RoboMaster S1 巡线项目

## 项目结构

```
speed/
├── 1_code/                 # 核心代码目录
│   ├── core/              # 主要机器人控制逻辑
│   │   └── robot_control.py
│   ├── config/            # 配置文件
│   │   └── line_params.yaml
│   ├── utils/             # 工具函数
│   │   └── image_processing.py
│   └── main.py           # 主入口点
│
├── 2_data/                # 数据存储
│   ├── calibration/      # 相机标定数据
│   └── log/             # 运行时日志
│
├── 3_assets/             # 资源文件
│   ├── track_design/     # 赛道布局和设计
│   └── docs/            # 文档 (此文件)
│
└── 4_test/               # 测试脚本
    ├── test_line_detection.py
    └── test_pid_control.py
```

## 主要特性

- **专业代码结构**: 模块化设计，关注点分离清晰
- **配置管理**: 所有参数在YAML文件中，便于调优
- **全面测试**: 每个组件都有独立的测试模块
- **详细文档**: 完整的注释和文档说明

## 快速开始

1. **安装依赖**:
   ```bash
   pip install robomaster pyyaml opencv-python numpy
   ```

2. **运行主程序**:
   ```bash
   cd 1_code
   python main.py
   ```

3. **运行测试**:
   ```bash
   cd 4_test
   python test_line_detection.py
   python test_pid_control.py
   ```

4. **参数管理**:
   ```bash
   # 列出所有参数
   python manage_params.py list
   
   # 获取特定参数
   python manage_params.py get pid.main.p
   
   # 设置参数值
   python manage_params.py set pid.main.p 0.3
   
   # 重置参数
   python manage_params.py reset pid.main.p
   ```

## 配置指南

所有可调参数都在 `1_code/config/line_params.yaml` 中，支持动态修改。关键参数：

### 颜色检测
- `color_detection.lower_blue/upper_blue`: 蓝色线条检测的HSV范围
- 根据光照条件和赛道颜色调整这些值

### PID 参数
- `pid.main`: 直线跟踪的参数
- `pid.curved`: 曲线段的参数
- 调整P、I、D值以获得最佳跟踪性能

### 速度设置
- `speed.base_straight`: 直线段的基础速度
- `speed.base_curve`: 曲线段的减速速度
- 根据赛道难度和机器人能力调整

## 调试技巧

1. **可视化调试**: 程序显示两个窗口：
   - "图像帧": 带有检测覆盖层的原始图像
   - "掩膜": 处理后的二进制掩膜

2. **参数调优**:
   - 从保守的速度开始
   - 逐步调整PID参数
   - 在不同赛道段测试

3. **日志分析**: 检查 `2_data/log/` 中的性能数据

## 贡献指南

1. 遵循模块化结构
2. 为新功能添加测试
3. 更改参数时更新文档
4. 使用类型提示和文档字符串

## 参数管理

项目提供了强大的参数管理系统：

### 命令行管理
使用 `manage_params.py` 工具：
```bash
# 列出所有参数
python manage_params.py list

# 获取参数值
python manage_params.py get pid.main.p

# 设置参数值  
python manage_params.py set speed.base_straight 0.9

# 重置参数
python manage_params.py reset color_detection.lower_blue
```

### 程序化接口
在代码中使用参数管理器：
```python
from config.parameter_interface import param_manager

# 获取参数
p_value = param_manager.get_parameter("pid.main.p")

# 设置参数
param_manager.set_parameter("speed.base_curve", 0.6)

# 重置所有参数
param_manager.reset_to_default()
```

### 动态配置
巡线机器人支持运行时配置更新：
```python
follower = LineFollower(use_dynamic_config=True)
follower.update_config_from_manager()  # 运行时更新配置
```

## 故障排除

- **连接问题**: 检查机器人WiFi连接
- **检测问题**: 调整配置中的HSV值
- **性能问题**: 调整PID参数和速度
- **参数问题**: 使用 `manage_params.py reset` 恢复默认设置
