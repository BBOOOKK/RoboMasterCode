from multi_robomaster import multi_robot


# 定义任务：前进 1 米
def forward_task(robot_group):
    robot_group.chassis.move(1, 0, 0, 2, 180).wait_for_completed()  # 前进 1 米


# 定义任务：后退 1 米
def backward_task(robot_group):
    robot_group.chassis.move(-1, 0, 0, 2, 180).wait_for_completed()  # 后退 1 米


if __name__ == '__main__':
    # 两台机器人的 SN 码
    robots_sn_list = ['3JKCJ6J00107HN', '3JKCHC600103E3']

    multi_robots = multi_robot.MultiEP()
    multi_robots.initialize()

    # 给机器人分配编号
    number = multi_robots.number_id_by_sn([0, robots_sn_list[0]],
                                          [1, robots_sn_list[1]])
    print("The number of robot is: {0}".format(number))

    # 建立一个包含两个机器人的组
    robot_group = multi_robots.build_group([0, 1])

    # 两台机器人并排前进 1 米
    multi_robots.run([robot_group, forward_task])

    # 两台机器人并排后退 1 米
    multi_robots.run([robot_group, backward_task])

    print("Game over")
    multi_robots.close()
