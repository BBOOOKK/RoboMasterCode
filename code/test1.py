import os
import sys
import importlib
import subprocess

def check_python_version():
    """检查Python版本是否兼容"""
    print("\n=== 检查Python版本 ===")
    version = sys.version_info
    if 3 <= version.major <= 3 and 6 <= version.minor <= 8:
        print(f"✅ 兼容的Python版本: {version.major}.{version.minor}.{version.micro}")
        return True
    else:
        print(f"❌ 不兼容的Python版本: {version.major}.{version.minor}.{version.micro}")
        print("   建议使用Python 3.6-3.8")
        return False

def check_robomaster_installed():
    """检查RoboMaster SDK是否安装及版本"""
    print("\n=== 检查RoboMaster SDK ===")
    try:
        robomaster = importlib.import_module("robomaster")
        print(f"✅ 已安装RoboMaster SDK: 版本 {robomaster.__version__}")
        
        # 检查关键配置项
        print("\n--- 检查SDK配置 ---")
        config = importlib.import_module("robomaster.config")
        required_configs = [
            "DEFAULT_CONN_PROTO", 
            "DEFAULT_UDP_PORT",
            "DEFAULT_TCP_PORT"
        ]
        config_ok = True
        for cfg in required_configs:
            if hasattr(config, cfg):
                print(f"✅ 存在配置项: {cfg} = {getattr(config, cfg)}")
            else:
                print(f"❌ 缺失配置项: {cfg}")
                config_ok = False
        
        return config_ok
    except ImportError:
        print("❌ 未安装RoboMaster SDK")
        print("   请执行: pip install robomaster")
        return False

def check_dependencies():
    """检查关键依赖库"""
    print("\n=== 检查依赖库 ===")
    dependencies = [
        "cv2",       # OpenCV (图像处理)
        "numpy",     # 数值计算
        "protobuf"   # 数据序列化
    ]
    all_ok = True
    for dep in dependencies:
        try:
            importlib.import_module(dep)
            print(f"✅ 已安装依赖: {dep}")
        except ImportError:
            print(f"❌ 缺失依赖: {dep}")
            print(f"   请执行: pip install {('opencv-python' if dep == 'cv2' else dep)}")
            all_ok = False
    return all_ok

def check_network():
    """检查与RoboMaster S1的网络连接环境"""
    print("\n=== 检查网络连接 ===")
    # 检查是否连接到S1的WiFi网段
    try:
        result = subprocess.check_output(
            "hostname -I | grep '192.168.2.'", 
            shell=True, 
            text=True
        ).strip()
        if result:
            print(f"✅ 已连接到S1的WiFi网段，本机IP: {result}")
        else:
            print("❌ 未连接到S1的WiFi热点")
            print("   请连接名称为 'RM_S1_xxxx' 的WiFi热点")
            return False
    except subprocess.CalledProcessError:
        print("❌ 未连接到S1的WiFi热点")
        print("   请连接名称为 'RM_S1_xxxx' 的WiFi热点")
        return False

    # 检查能否ping通S1
    try:
        subprocess.check_output(
            "ping -c 1 -W 2 192.168.2.1 > /dev/null 2>&1",
            shell=True
        )
        print("✅ 成功ping通S1 (192.168.2.1)")
        return True
    except subprocess.CalledProcessError:
        print("❌ 无法ping通S1 (192.168.2.1)")
        print("   请检查S1是否处于AP模式，或重启S1后重试")
        return False

def check_udev_rules():
    """检查Linux设备权限配置（针对USB连接）"""
    print("\n=== 检查设备权限 ===")
    if os.path.exists("/etc/udev/rules.d/99-robomaster.rules"):
        print("✅ 已存在RoboMaster设备权限规则")
        return True
    else:
        print("⚠️ 未找到RoboMaster设备权限规则（USB连接可能需要）")
        print("   可忽略此警告（仅影响USB连接方式）")
        return True  # 非致命错误

def main():
    print("===== RoboMaster 环境检测工具 =====")
    
    # 依次执行检查项
    checks = [
        ("Python版本", check_python_version),
        ("RoboMaster SDK", check_robomaster_installed),
        ("依赖库", check_dependencies),
        ("网络连接", check_network),
        ("设备权限", check_udev_rules)
    ]
    
    # 统计检查结果
    all_passed = True
    for name, check_func in checks:
        if not check_func():
            all_passed = False
    
    # 输出最终结果
    print("\n===== 检测结果总结 =====")
    if all_passed:
        print("🎉 所有关键项检查通过，环境基本正常！")
    else:
        print("❌ 部分检查项未通过，请根据提示修复后重试")

if __name__ == "__main__":
    main()
    
