from robomaster import robot

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_chassis.move(x=0, y=0, z=-90, xy_speed=0.7,z_speed=50).wait_for_completed()#顺时针旋转90度
    for i in range(5):#重复5次
        ep_chassis.move(x=0.75, y=0, z=0, xy_speed=0.7,z_speed=50).wait_for_completed()#机器人向前运动0.75米
        ep_chassis.move(x=0, y=0, z=72, xy_speed=0.7,z_speed=50).wait_for_completed()#逆时针旋转72°
        ep_chassis.move(x=0.75, y=0, z=0, xy_speed=0.7,z_speed=50).wait_for_completed()#机器人向前运动0.75米
        ep_chassis.move(x=0, y=0, z=-144, xy_speed=0.7,z_speed=50).wait_for_completed()#顺时针旋转144°
    ep_robot.close()
