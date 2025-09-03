from robomaster import robot
import time

def avoid_obstacle(chassis, distance_threshold=0.3):
    """当检测到前方障碍物距离小于阈值（米）时，自动转向避开"""
    while True:
        # 获取超声波传感器数据（需机器人支持，不同型号接口可能不同）
        distance = ep_robot.sensor.get_ultrasonic_distance()  # 单位：米
        print(f"前方距离: {distance:.2f}米")
        
        if distance < distance_threshold:
            print("检测到障碍物，开始转向...")
            # 停止前进
            chassis.stop()
            time.sleep(0.5)
            # 向右旋转30度
            chassis.move(x=0, y=0, z=-30, speed=60).wait_for_completion()
            time.sleep(0.5)
            # 前进一段距离避开
            chassis.move(x=0.4, y=0, z=0, speed=0.3).wait_for_completion()
            break  # 避开后退出循环
        else:
            # 正常前进
            chassis.move(x=0.1, y=0, z=0, speed=0.2).wait_for_completion()
        time.sleep(0.5)

if __name__ == '__main__':
    ep_robot = robot.Robot(ip="192.168.10.2")
    ep_robot.initialize()
    chassis = ep_robot.chassis
    
    print("开始避障测试...")
    avoid_obstacle(chassis, distance_threshold=0.3)  # 距离小于30cm时避障
    
    ep_robot.close()
    print("避障测试结束")
    