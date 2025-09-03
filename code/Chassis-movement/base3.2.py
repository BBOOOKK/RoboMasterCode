from robomaster import robot
import time

if __name__ == '__main__':
    ep_robot = robot.Robot(ip="192.168.10.2")
    ep_robot.initialize()
    chassis = ep_robot.chassis
    
    print("开始圆形轨迹运动...")
    # 左轮速度低于右轮，形成顺时针圆形（速度差越小，半径越大）
    chassis.drive_wheels(w1=40, w2=40, w3=60, w4=60)  # 具体参数需根据机器人调试
    time.sleep(5)  # 持续5秒
    chassis.drive_wheels(0, 0, 0, 0)  # 停止
    
    ep_robot.close()
    print("圆形轨迹运动结束")