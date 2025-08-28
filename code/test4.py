from robomaster import robot
import time
import os

def check_network():
    """检查与S1的网络连接"""
    try:
        # 检查是否在S1的WiFi网段
        ip_output = os.popen("hostname -I | grep '192.168.2.'").read().strip()
        if not ip_output:
            print("未连接到S1的WiFi热点（RM_S1_xxxx）")
            return False
        
        # 检查能否ping通S1
        ping_result = os.system("ping -c 1 -W 2 192.168.2.1 > /dev/null 2>&1")
        if ping_result != 0:
            print("无法ping通S1，请确认机器人处于AP模式")
            return False
        
        return True
    except Exception as e:
        print(f"网络检查错误：{e}")
        return False

def main():
    ep_robot = robot.Robot()
    connected = False

    try:
        if not check_network():
            return

        # 连接机器人
        ep_robot.initialize(ip="192.168.2.1")
        connected = True
        print("S1连接成功！")

        # 简单动作演示
        print("底盘轻微移动...")
        ep_robot.chassis.drive_speed(x=0.1, y=0, z=0, timeout=1)
        time.sleep(1)
        ep_robot.chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(1)

    except Exception as e:
        print(f"操作错误：{e}")
    finally:
        if connected:
            ep_robot.close()
            print("连接已关闭")

if __name__ == "__main__":
    main()

