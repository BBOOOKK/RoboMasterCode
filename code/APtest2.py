from robomaster import robot
import time

if __name__ == '__main__':
    # 连接机器人（根据连接方式修改IP）
    # 无线热点模式默认IP: 192.168.10.2
    # 有线连接默认IP: 192.168.2.1
    ep = robot.Robot(ip="192.168.10.2")
    
    # 初始化
    ep.initialize()
    
    # 测试云台（如果有）
    ep.gimbal.recenter().wait_for_completion()
    time.sleep(1)
    
    # 获取电池信息
    battery_info = ep.battery.get_battery_info()
    print(f"电池信息: {battery_info}")
    
    # 关闭连接
    ep.close()
    print("连接已关闭")