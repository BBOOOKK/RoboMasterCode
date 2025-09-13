"""
参数接口管理模块
提供统一的参数修改接口，支持YAML配置文件和运行时动态修改
"""

import yaml
from pathlib import Path
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass, asdict, fields
import numpy as np


# Define the configuration structure here to avoid circular imports
@dataclass
class LineFollowerConfig:
    """巡线机器人配置参数"""
    # 连接设置
    connection_type: str = "ap"
    
    # 颜色检测设置
    lower_blue: Tuple[int, int, int] = (100, 80, 50)
    upper_blue: Tuple[int, int, int] = (130, 255, 255)
    
    # ROI 区域设置 - 扩大检测范围
    roi_row_offsets: Tuple[int, int, int, int, int] = (150, 120, 90, 60, 30)  # 从远到近
    min_white_pixels: int = 8
    
    # 未来线路预测设置
    prediction_distance: int = 50  # 预测未来50像素距离
    early_curve_threshold: float = 0.15  # 提前检测弯道的曲率阈值
    preemptive_slowdown_factor: float = 0.7  # 提前降速系数
    
    # PID 控制器设置 - 超激进弯道控制
    main_pid_params: Tuple[float, float, float, float] = (0.5, 0.0006, 0.15, 130)     # 进一步提高响应
    curved_pid_params: Tuple[float, float, float, float] = (2.5, 0.001, 0.45, 280)    # 极强弯道控制，防止转出
    
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
    
    # 速度设置 - 提升直线速度，增强弯道灵敏度
    base_speed_straight: float = 1.55         # 提升直线速度
    base_speed_curve: float = 0.6          # 适当提升弯道基础速度
    
    # 曲率阈值设置 - 大幅降低阈值，更敏感检测弯道
    curvature_enter_threshold: float = 0.08   # 大幅降低阈值，极早检测弯道
    curvature_exit_threshold: float = 0.05    # 大幅降低退出阈值，保持弯道模式


class ParameterManager:
    """参数管理器类，提供统一的参数访问和修改接口"""
    
    def __init__(self, config_path: str = "line_params.yaml"):
        """
        初始化参数管理器
        
        参数:
            config_path: 配置文件路径
        """
        self.config_path = Path(__file__).parent / config_path
        self.default_config = LineFollowerConfig()
        self.current_config = self._load_config()
    
    def _load_config(self) -> LineFollowerConfig:
        """从YAML文件加载配置"""
        if self.config_path.exists():
            try:
                with open(self.config_path, 'r', encoding='utf-8') as f:
                    config_data = yaml.safe_load(f)
                return self._dict_to_config(config_data)
            except Exception as e:
                print(f"加载配置文件失败，使用默认配置: {e}")
                return LineFollowerConfig()
        else:
            print("配置文件不存在，创建默认配置")
            self._save_config(self.default_config)
            return LineFollowerConfig()
    
    def _save_config(self, config: LineFollowerConfig) -> None:
        """保存配置到YAML文件"""
        config_dict = self._config_to_dict(config)
        with open(self.config_path, 'w', encoding='utf-8') as f:
            yaml.dump(config_dict, f, allow_unicode=True, sort_keys=False)
    
    def _config_to_dict(self, config: LineFollowerConfig) -> Dict[str, Any]:
        """将配置对象转换为字典"""
        config_dict = {}
        for field in fields(config):
            value = getattr(config, field.name)
            if hasattr(value, '__dict__') or isinstance(value, (list, tuple, dict)):
                config_dict[field.name] = value
            else:
                config_dict[field.name] = value
        return config_dict
    
    def _dict_to_config(self, config_dict: Dict[str, Any]) -> LineFollowerConfig:
        """将字典转换为配置对象"""
        return LineFollowerConfig(**config_dict)
    
    def get_all_parameters(self) -> Dict[str, Any]:
        """获取所有参数的字典形式"""
        return self._config_to_dict(self.current_config)
    
    def get_parameter(self, param_path: str) -> Any:
        """
        获取指定参数的值
        
        参数:
            param_path: 参数路径，支持点符号访问嵌套参数
                      例如: "pid.main.p", "color_detection.lower_blue"
        
        返回:
            参数值
        """
        try:
            parts = param_path.split('.')
            value = self.get_all_parameters()
            for part in parts:
                if isinstance(value, dict) and part in value:
                    value = value[part]
                else:
                    raise KeyError(f"参数 '{part}' 不存在")
            return value
        except (KeyError, AttributeError) as e:
            raise ValueError(f"无法获取参数 '{param_path}': {e}")
    
    def set_parameter(self, param_path: str, value: Any) -> None:
        """
        设置指定参数的值
        
        参数:
            param_path: 参数路径，支持点符号访问嵌套参数
            value: 要设置的值
        """
        try:
            # 创建配置的深拷贝
            config_dict = self.get_all_parameters()
            
            # 解析参数路径
            parts = param_path.split('.')
            current = config_dict
            
            # 导航到父级
            for part in parts[:-1]:
                if part not in current:
                    current[part] = {}
                current = current[part]
            
            # 设置值
            last_part = parts[-1]
            current[last_part] = value
            
            # 转换回配置对象并验证
            new_config = self._dict_to_config(config_dict)
            self.current_config = new_config
            
            # 保存到文件
            self._save_config(new_config)
            
            print(f"参数 '{param_path}' 已设置为: {value}")
            
        except Exception as e:
            raise ValueError(f"设置参数 '{param_path}' 失败: {e}")
    
    def reset_to_default(self, param_path: Optional[str] = None) -> None:
        """
        重置参数到默认值
        
        参数:
            param_path: 要重置的参数路径，如果为None则重置所有参数
        """
        if param_path is None:
            self.current_config = LineFollowerConfig()
            self._save_config(self.current_config)
            print("所有参数已重置为默认值")
        else:
            default_value = self.get_parameter_from_default(param_path)
            self.set_parameter(param_path, default_value)
            print(f"参数 '{param_path}' 已重置为默认值: {default_value}")
    
    def get_parameter_from_default(self, param_path: str) -> Any:
        """从默认配置获取参数值"""
        default_dict = self._config_to_dict(self.default_config)
        parts = param_path.split('.')
        value = default_dict
        for part in parts:
            value = value[part]
        return value
    
    def list_all_parameters(self) -> None:
        """列出所有可用的参数"""
        print("可用参数列表:")
        print("=" * 50)
        
        def print_params(data: Dict[str, Any], prefix: str = ""):
            for key, value in data.items():
                full_path = f"{prefix}.{key}" if prefix else key
                if isinstance(value, dict):
                    print_params(value, full_path)
                else:
                    default_value = self.get_parameter_from_default(full_path)
                    current_value = self.get_parameter(full_path)
                    print(f"{full_path}:")
                    print(f"  当前值: {current_value}")
                    print(f"  默认值: {default_value}")
                    print()
        
        print_params(self.get_all_parameters())
    
    def update_from_file(self) -> None:
        """从配置文件重新加载参数"""
        self.current_config = self._load_config()
        print("参数已从配置文件更新")
    
    def export_config(self, export_path: str) -> None:
        """
        导出当前配置到指定文件
        
        参数:
            export_path: 导出文件路径
        """
        export_path = Path(export_path)
        with open(export_path, 'w', encoding='utf-8') as f:
            yaml.dump(self.get_all_parameters(), f, allow_unicode=True, sort_keys=False)
        print(f"配置已导出到: {export_path}")


# 全局参数管理器实例
param_manager = ParameterManager()


def main():
    """参数管理接口的测试主函数"""
    print("RoboMaster S1 参数管理接口")
    print("=" * 50)
    
    # 创建参数管理器
    manager = ParameterManager()
    
    while True:
        print("\n选择操作:")
        print("1. 列出所有参数")
        print("2. 获取参数值")
        print("3. 设置参数值")
        print("4. 重置参数")
        print("5. 从文件更新")
        print("6. 导出配置")
        print("7. 退出")
        
        choice = input("请输入选择 (1-7): ").strip()
        
        if choice == '1':
            manager.list_all_parameters()
            
        elif choice == '2':
            param_path = input("请输入参数路径: ").strip()
            try:
                value = manager.get_parameter(param_path)
                print(f"{param_path} = {value}")
            except ValueError as e:
                print(f"错误: {e}")
                
        elif choice == '3':
            param_path = input("请输入参数路径: ").strip()
            try:
                current_value = manager.get_parameter(param_path)
                print(f"当前值: {current_value}")
                
                new_value_str = input("请输入新值: ").strip()
                # 尝试转换类型
                try:
                    if isinstance(current_value, (list, tuple)):
                        new_value = eval(new_value_str)
                    elif isinstance(current_value, int):
                        new_value = int(new_value_str)
                    elif isinstance(current_value, float):
                        new_value = float(new_value_str)
                    elif isinstance(current_value, bool):
                        new_value = new_value_str.lower() in ('true', '1', 'yes')
                    else:
                        new_value = new_value_str
                        
                    manager.set_parameter(param_path, new_value)
                except (ValueError, SyntaxError):
                    manager.set_parameter(param_path, new_value_str)
                    
            except ValueError as e:
                print(f"错误: {e}")
                
        elif choice == '4':
            param_path = input("请输入要重置的参数路径 (直接回车重置所有): ").strip()
            if param_path:
                manager.reset_to_default(param_path)
            else:
                manager.reset_to_default()
                
        elif choice == '5':
            manager.update_from_file()
            
        elif choice == '6':
            export_path = input("请输入导出文件路径: ").strip()
            manager.export_config(export_path)
            
        elif choice == '7':
            print("退出参数管理")
            break
            
        else:
            print("无效选择，请重新输入")


if __name__ == "__main__":
    main()
