from multi_robomaster import multi_robot

def task1(robot_group):
    robot_group.chassis.move(0, -1, 0, 2, 180).wait_for_completed()#平移1米动作函数
	
def task2(robot_group):
    robot_group.chassis.move(0,  1, 0, 2, 180).wait_for_completed()#向另一侧平移1米动作函数
    
if __name__ == '__main__':
    robots_sn_list = ['3JKDH2T0017D2Z', '3JKDH4V001SW8J','3JKDH3B0019574','3JKDH2T001Y75Q']#四台设备的SN码
    multi_robots = multi_robot.MultiEP()
    multi_robots.initialize()
    number = multi_robots.number_id_by_sn([0, robots_sn_list[0]], [1, robots_sn_list[1]],
										[2, robots_sn_list[2]],[3, robots_sn_list[3]])#按照SN码的顺序，对四台进行编号，按照摆放位置
    print("The number of robot is: {0}".format(number))
    
	
    robot_group1 = multi_robots.build_group([0,2])#0号与2号为第一组
    robot_group2 = multi_robots.build_group([1,3])#1号与3号为第二组

	
    multi_robots.run([robot_group1, task1])#第一组运行任务一的动作函数
    multi_robots.run([robot_group2, task2])#第二组运行任务二的动作函数
	
    print("Game over")
    multi_robots.close()