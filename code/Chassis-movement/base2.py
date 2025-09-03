from robomaster import robot
import time

if __name__ == '__main__':
    ep_robot = robot.Robot(ip="192.168.10.2")
    ep_robot.initialize()
    chassis = ep_robot.chassis
    
    print("开始连续运动测试...")
    
    # 1. 左右轮同速前进（50%速度）
    chassis.drive_wheels(w1=50, w2=50, w3=50, w4=50)  # w1~w4对应四个轮子（具体取决于机器人型号）
    time.sleep(2)  # 持续2秒
    
    # 2. 减速前进（30%速度）
    chassis.drive_wheels(w1=30, w2=30, w3=30, w4=30)
    time.sleep(1.5)
    
    # 3. 原地顺时针旋转（左轮正转，右轮反转）
    chassis.drive_wheels(w1=40, w2=40, w3=-40, w4=-40)
    time.sleep(1.5)
    
    # 4. 向右平移（对角线轮子同速，方向相反）
    chassis.drive_wheels(w1=40, w2=-40, w3=40, w4=-40)
    time.sleep(1.5)
    
    # 5. 停止所有运动
    chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    
    ep_robot.close()
    print("连续运动测试结束")