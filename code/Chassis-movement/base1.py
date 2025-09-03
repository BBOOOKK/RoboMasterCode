from robomaster import robot
import time

if __name__ == '__main__':
    # 连接机器人（根据实际连接方式修改IP）
    ep_robot = robot.Robot(ip="192.168.10.2")
    ep_robot.initialize()
    
    # 获取底盘控制对象
    chassis = ep_robot.chassis
    
    print("开始基础运动测试...")
    
    # 1. 前进50cm（x轴方向，单位：米）
    chassis.move(x=0.5, y=0, z=0, speed=0.3).wait_for_completion()  # speed为移动速度（米/秒）
    time.sleep(1)
    
    # 2. 后退30cm（x轴负方向）
    chassis.move(x=-0.3, y=0, z=0, speed=0.3).wait_for_completion()
    time.sleep(1)
    
    # 3. 向右平移40cm（y轴正方向）
    chassis.move(x=0, y=0.4, z=0, speed=0.3).wait_for_completion()
    time.sleep(1)
    
    # 4. 向左平移20cm（y轴负方向）
    chassis.move(x=0, y=-0.2, z=0, speed=0.3).wait_for_completion()
    time.sleep(1)
    
    # 5. 顺时针旋转90度（z轴负方向，单位：度）
    chassis.move(x=0, y=0, z=-90, speed=60).wait_for_completion()  # speed为旋转速度（度/秒）
    time.sleep(1)
    
    # 6. 逆时针旋转90度（z轴正方向）
    chassis.move(x=0, y=0, z=90, speed=60).wait_for_completion()
    time.sleep(1)
    
    # 结束并关闭连接
    ep_robot.close()
    print("基础运动测试结束")