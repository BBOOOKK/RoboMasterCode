from robomaster import robot
import time
import os

def check_s1_network():
    """检查是否连接到S1的WiFi热点并能通信"""
    try:
        # 检查IP是否在S1的AP网段
        ip_result = os.popen("hostname -I | grep '192.168.2.'").read().strip()
        if not ip_result:
            print("网络错误：未连接到S1的WiFi热点（RM_S1_xxxx）")
            return False
        
        # 检查能否ping通S1
        ping_result = os.system("ping -c 1 -W 2 192.168.2.1 > /dev/null 2>&1")
        if ping_result != 0:
            print("网络错误：无法与S1通信，请检查机器人是否开启并处于AP模式")
            return False
        
        print("网络连接正常")
        return True
    except Exception as e:
        print(f"网络检查失败：{str(e)}")
        return False

def s1_old_sdk_demo():
    # 初始化机器人对象（旧版SDK通用写法）
    ep_robot = robot.Robot()
    is_connected = False

    try:
        # 1. 先检查网络环境
        if not check_s1_network():
            return

        # 2. 旧版SDK连接方式（无conn_type参数，显式指定IP）
        print("尝试连接S1机器人...")
        ep_robot.initialize(ip="192.168.2.1")  # 旧版需显式指定S1的IP
        is_connected = True
        print("S1连接成功！")

        # 3. 基础控制演示（使用旧版兼容接口）
        # 3.1 底盘控制（使用drive_speed而非move，旧版更稳定）
        print("底盘前进1秒...")
        ep_robot.chassis.drive_speed(x=0.2, y=0, z=0, timeout=1)  # x:前进速度(m/s)
        time.sleep(1)
        ep_robot.chassis.drive_speed(x=0, y=0, z=0)  # 停止
        time.sleep(0.5)
