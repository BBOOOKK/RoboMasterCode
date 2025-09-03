from robomaster import robot
import time

if __name__ == '__main__':
    # === 初始化机器人 ===
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    # === 参数设定（稍快） ===
    LINEAR_SPEED = 0.7   # m/s
    ANGULAR_SPEED = 120  # °/s

    # === 1. 三角形轨迹 ===
    print("开始执行三角形轨迹...")
    for i in range(3):
        ep_chassis.move(x=1, y=0, z=0, xy_speed=LINEAR_SPEED).wait_for_completed()
        ep_chassis.move(x=0, y=0, z=-120, z_speed=ANGULAR_SPEED).wait_for_completed()
    time.sleep(1)

    # === 2. 正方形轨迹 ===
    print("开始执行正方形轨迹...")
    directions = [0, 90, 180, -90]
    for d in directions:
        ep_chassis.move(x=1, y=0, z=0, xy_speed=LINEAR_SPEED, angle=d).wait_for_completed()
    time.sleep(1)

    # === 3. 圆形轨迹 ===
    print("开始执行圆形轨迹...")
    # 模拟差速圆周，左轮慢，右轮快
    ep_chassis.drive_wheels(w1=30, w2=100, w3=30, w4=100)
    time.sleep(4.5)   # 时间缩短，转一圈更快
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    time.sleep(1)

    # === 4. U-turn Drift ===
    print("开始执行U-turn漂移...")
    # 初始直行
    ep_chassis.drive_wheels(w1=360, w2=360, w3=360, w4=360)
    time.sleep(2.0)   # 比文档快
    # 漂移转弯
    ep_chassis.drive_wheels(w1=360, w2=-360, w3=360, w4=-360)
    time.sleep(0.4)
    # 停止
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    time.sleep(1)

    # === 5. Slide Dodge ===
    print("开始执行Slide Dodge...")
    # 循环几次左右闪避
    for i in range(3):
        ep_chassis.move(x=0.5, y=0, z=90, xy_speed=LINEAR_SPEED, z_speed=ANGULAR_SPEED).wait_for_completed()
        time.sleep(0.8)
        ep_chassis.move(x=0.5, y=0, z=-90, xy_speed=LINEAR_SPEED, z_speed=ANGULAR_SPEED).wait_for_completed()
        time.sleep(0.8)

    # === 结束 ===
    ep_robot.close()
    print("任务完成")