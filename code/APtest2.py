from robomaster import robot

if __name__ == '__main__':
    # 初始化机器人（AP模式）
    ep_robot = robot.Robot()
    try:
        # 尝试连接，增加超时参数（可选）
        ep_robot.initialize(conn_type='ap', timeout=10)
        print("机器人连接成功！")
        
        # 测试：让机器人发出声音
        ep_robot.play_sound(sound=1)
        
    except Exception as e:
        print(f"连接失败：{e}")
    finally:
        # 断开连接
        ep_robot.close()