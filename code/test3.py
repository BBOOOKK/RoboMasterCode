from robomaster import robot
import time
import os

def check_s1_network():
    """检查与S1的网络连接状态"""
    try:
        # 检查是否在S1的WiFi网段
        ip_info = os.popen("hostname -I | grep '192.168.2.'").read().strip()
        if not ip_info:
            print("错误：未连接到S1的WiFi热点（名称格式：RM_S1_xxxx）")
            return False
        
        # 检查能否ping通S1
        ping_result = os.system("ping -c 1 -W 2 192.168.2.1 > /dev/null 2>&1")
        if ping_result != 0:
            print("错误：无法ping通S1，请确认机器人已开启并处于AP模式（蓝灯闪烁）")
            return False
        
        print("网络连接正常，准备连接机器人...")
        return True
    except Exception as e:
        print(f"网络检查失败：{str(e)}")
        return False

def main():
    # 初始化机器人对象
    ep_robot = robot.Robot()
    connected = False

    try:
        # 先检查网络
        if not check_s1_network():
            return

        # 旧版SDK连接方式（显式指定IP）
        ep_robot.initialize(ip="192.168.2.1")
        connected = True
        print("S1机器人连接成功！")

        # 基础控制演示
        print("\n=== 开始基础控制演示 ===")
        
        # 1. 底盘前进0.5秒
        print("底盘前进...")
        ep_robot.chassis.drive_speed(x=0.2, y=0, z=0, timeout=0.5)
        time.sleep(0.5)
        ep_robot.chassis.drive_speed(x=0, y=0, z=0)  # 停止
        time.sleep(1)

        # 2. 底盘旋转90度
        print("底盘旋转...")
        ep_robot.chassis.drive_speed(x=0, y=0, z=30, timeout=3)  # 30度/秒
        time.sleep(3)
        ep_robot.chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(1)

        # 3. 云台复位
        print("云台复位...")
        ep_robot.gimbal.recenter().wait_for_completed()
        time.sleep(1)

        # 4. 读取电池电量
        try:
            battery = ep_robot.battery.get_battery_percentage()
            print(f"当前电池电量：{battery}%")
        except AttributeError:
            print("提示：旧版SDK不支持电量读取，已跳过")

        # 5. 灯光控制
        print("灯光测试...")
        ep_robot.led.set_led(r=255, g=0, b=0, led="all")  # 红灯
        time.sleep(1)
        ep_robot.led.set_led(r=0, g=255, b=0, led="all")  # 绿灯
        time.sleep(1)
        ep_robot.led.set_led(r=0, g=0, b=255, led="all")  # 蓝灯
        time.sleep(1)

        print("\n=== 演示结束 ===")

    except Exception as e:
        print(f"发生错误：{str(e)}")

    finally:
        # 确保断开连接
        if connected:
            print("\n断开与S1的连接...")
            ep_robot.close()
        else:
            print("\n未建立有效连接，程序退出")

if __name__ == "__main__":
    main()

