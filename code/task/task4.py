from robomaster import robot
import time
import math

class SectorDodgeController:
    def __init__(self):
        # 初始化机器人
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize()
        # 获取底盘控制对象
        self.chassis = self.ep_robot.chassis
        # 动作超时时间（秒）
        self.action_timeout = 3.0

        # 圆弧运动参数（可根据需求调整）
        self.radius = 0.8  # 圆弧半径（米）
        self.angular_velocity = 30  # 角速度（度/秒）
        self.linear_velocity = self.calc_linear_velocity()  # 线速度（米/秒）
        
        # 控制参数
        self.control_frequency = 20  # 控制频率（Hz）
        self.control_interval = 1.0 / self.control_frequency  # 控制间隔（秒）

    def calc_linear_velocity(self):
        """根据角速度和半径计算线速度：v = ω * r（需将角速度转换为弧度/秒）"""
        angular_rad = self.angular_velocity * (math.pi / 180)  # 度转弧度
        return angular_rad * self.radius

    def set_robot_mode(self):
        """设置机器人模式为自由模式，保持朝向不变"""
        self.ep_robot.set_robot_mode(mode='free')
        print(f"机器人模式：自由模式 | 半径：{self.radius}m | 角速度：{self.angular_velocity}°/s | 线速度：{self.linear_velocity:.2f}m/s")

    def _move_arc(self, angle, clockwise=True):
        """
        执行弧形平移移动（保持机器人朝向不变）
        :param angle: 旋转角度（度）
        :param clockwise: 是否顺时针
        """
        # 计算移动时间（角度/角速度）
        move_time = abs(angle) / self.angular_velocity
        # 计算总控制次数
        control_count = int(move_time / self.control_interval)
        # 计算每次控制的角度变化量
        angle_step = angle / control_count
        angle_step_rad = angle_step * (math.pi / 180)  # 转换为弧度
        
        print(f"开始圆弧移动: 角度{angle}度, 方向{'顺时针' if clockwise else '逆时针'}, 时间{move_time:.2f}秒")
        
        start_time = time.time()
        
        for i in range(control_count):
            # 计算当前角度（弧度）
            current_angle = (i + 1) * angle_step_rad
            if clockwise:
                current_angle = -current_angle  # 顺时针为负
                
            # 计算x和y方向的速度分量
            vx = self.linear_velocity * math.cos(current_angle)
            vy = self.linear_velocity * math.sin(current_angle)
            
            # 设置底盘速度（保持z=0以确保不旋转）
            self.chassis.drive_speed(x=vx, y=vy, z=0)
            
            # 等待控制间隔
            time.sleep(self.control_interval)
        
        # 确保移动时间准确
        elapsed_time = time.time() - start_time
        if elapsed_time < move_time:
            time.sleep(move_time - elapsed_time)
            
        # 停止底盘
        self.chassis.drive_speed(x=0, y=0, z=0)
        return True

    def sector_dodge(self):
        """执行完整扇形闪避动作：左右各划一段圆弧"""
        print("开始扇形闪避动作...")
        # 1. 向右前方移动（45度方向）
        self._move_arc(angle=60, clockwise=False)
        time.sleep(0.5)
        # 2. 向左前方移动（135度方向）
        self._move_arc(angle=120, clockwise=True)
        time.sleep(0.5)
        # 3. 向右前方移动（回到初始方向）
        self._move_arc(angle=60, clockwise=False)
        print("扇形闪避动作完成！")

    def close(self):
        """关闭机器人连接"""
        self.chassis.drive_speed(x=0, y=0, z=0)  # 确保停止
        self.ep_robot.close()

if __name__ == "__main__":
    controller = SectorDodgeController()
    try:
        controller.set_robot_mode()
        controller.sector_dodge()
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        controller.close()