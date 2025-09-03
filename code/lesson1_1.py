from robomaster import robot
import time

if __name__ == '__main__':
    # 初始化机器人
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis


    # === 三角形 ===
    print("开始执行三角形轨迹...")
    for i in range(3):
        ep_chassis.move(x=1, y=0, z=0, xy_speed=10).wait_for_completed()
        ep_chassis.move(x=0, y=0, z=-120, z_speed=100).wait_for_completed()
    time.sleep(1)

    # === 正方形 ===
    print("开始执行正方形轨迹...")
    directions = [0, 90, 180, -90]
    for d in directions:
        ep_chassis.move(x=1, y=0, z=0, xy_speed=10, angle=d).wait_for_completed()
    time.sleep(1)

    # === 圆形 ===
    print("开始执行圆形轨迹...")
    # 左轮 10rpm，右轮 100rpm，保持6秒（此处简化为 yaw 转动+前进）
    ep_chassis.drive_wheels(w1=10, w2=100, w3=10, w4=100)
    time.sleep(6)
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

    # 结束
    ep_robot.close()
    print("任务完成")