import robomaster
from robomaster import robot


if __name__ == '__main__':
    # 如果本地IP 自动获取不正确，手动指定本地IP地址
    # robomaster.config.LOCAL_IP_STR = "192.168.2.20"
    ep_robot = robot.Robot()

    # 指定连接方式为AP 直连模式
    ep_robot.initialize(conn_type='ap')

    version = ep_robot.get_version()
    print("Robot version: {0}".format(version))
    ep_robot.close()



