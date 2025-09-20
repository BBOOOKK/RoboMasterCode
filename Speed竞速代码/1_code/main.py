"""
RoboMaster S1 巡线程序主入口点
此文件是启动巡线过程的主要接口
"""

import sys
from core.robot_control import LineFollower, LineFollowerConfig

def main():
    """巡线应用程序的主入口点"""
    print("启动 RoboMaster S1 巡线程序...")
    print("按 'q' 键退出")

    try:
        # 尝试不同的连接方式
        connection_types = ["ap", "sta"]
        
        for conn_type in connection_types:
            try:
                print(f"尝试 {conn_type.upper()} 模式连接...")
                config = LineFollowerConfig(connection_type=conn_type)
                follower = LineFollower(config)
                print("连接成功！")
                follower.run()
                break
            except Exception as e:
                print(f"{conn_type.upper()} 模式连接失败: {e}")
                if conn_type == connection_types[-1]:
                    print("所有连接模式都失败了")
                    raise
                continue
                
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行出错: {e}")
        print("\n连接故障排除:")
        print("1. 确保RoboMaster S1已开机")
        print("2. 确保电脑连接到机器人的WiFi网络")
        print("3. AP模式: 连接到机器人热点 (通常为 RoboMaster_XXXX)")
        print("4. STA模式: 确保机器人和电脑在同一网络")
        print("5. 检查网络防火墙设置")
    finally:
        print("程序结束")


if __name__ == "__main__":
    main()
