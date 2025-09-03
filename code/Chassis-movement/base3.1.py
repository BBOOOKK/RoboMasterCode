
from robomaster import robot
import time

def move_square(chassis, side_length=0.5):
    """沿正方形轨迹运动，side_length为边长（米）"""
    for _ in range(4):
        # 前进一段距离（边长）
        chassis.move(x=side_length, y=0, z=0, speed=0.3).wait_for_completion()
        time.sleep(0.5)
        # 右转90度
        chassis.move(x=0, y=0, z=-90, speed=60).wait_for_completion()
        time.sleep(0.5)

if __name__ == '__main__':
    ep_robot = robot.Robot(ip="192.168.10.2")
    ep_robot.initialize()
    chassis = ep_robot.chassis
    
    print("开始正方形轨迹运动...")
    move_square(chassis, side_length=0.6)  # 边长60cm的正方形
    
    ep_robot.close()
    print("正方形轨迹运动结束")