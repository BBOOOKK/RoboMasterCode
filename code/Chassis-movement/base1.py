from robomaster import robot
import time

def _wait_action(action, timeout=None):
    """兼容不同 SDK 的 action 等待接口。"""
    if action is None:
        return
    # 常见的等待方法名集合
    names = ("wait_for_completion", "wait_for_completed", "wait", "join", "result")
    for name in names:
        if hasattr(action, name):
            fn = getattr(action, name)
            try:
                if callable(fn):
                    # 尝试以 timeout 调用，否则不带参数调用
                    try:
                        if timeout is not None:
                            return fn(timeout)
                        return fn()
                    except TypeError:
                        return fn()
                else:
                    return fn
            except Exception:
                # 如果某个方法抛异常，继续尝试其它方法
                continue
    # 尝试通过轮询常见状态字段判断完成
    end_flags = ("is_completed", "is_finished", "done")
    if any(hasattr(action, f) for f in end_flags):
        end_time = time.time() + (timeout if timeout else 5)
        while time.time() < end_time:
            if getattr(action, "is_completed", False) or getattr(action, "is_finished", False) or getattr(action, "done", False):
                return
            time.sleep(0.1)
    # 无法判断时直接返回（保守处理）
    return

def safe_move(chassis, x=0, y=0, z=0, speed=None, wait=True, timeout=None):
    """
    兼容不同版本 SDK 的 chassis.move 调用并可选等待完成：
    - 尝试关键字 speed -> 位置参数 speed -> 不带 speed 调用。
    - 如果 wait=True，会调用兼容等待函数等待动作完成。
    返回 move() 的结果（可能为 None 或 action 对象）。
    """
    move = chassis.move
    action = None
    try:
        if speed is not None:
            action = move(x=x, y=y, z=z, speed=speed)
        else:
            action = move(x=x, y=y, z=z)
    except TypeError:
        try:
            if speed is not None:
                action = move(x, y, z, speed)
            else:
                action = move(x, y, z)
        except TypeError:
            # 最后兜底：只传位置/角度，不带 speed
            action = move(x=x, y=y, z=z)

    if wait:
        _wait_action(action, timeout=timeout)
    return action

# 新增：平滑速度控制与分段退回策略
def _move_towards(cur, target, max_delta):
    if abs(target - cur) <= max_delta:
        return target
    return cur + max_delta if target > cur else cur - max_delta

def smooth_move_velocity(chassis, vx=0.0, vy=0.0, yaw_rate=0.0, duration=1.0, max_accel=0.5, step=0.05):
    """
    尝试以速度控制平滑移动（vx,vy 单位 m/s，yaw_rate 单位 deg/s 或 SDK 要求的角速度单位）。
    - 优先使用 chassis.drive_speed / chassis.drive；
    - 按 max_accel (m/s^2 或 deg/s^2) 逐步加速/减速；
    - duration 指总持续时间（秒）。
    返回 True 表示使用速度控制；False 表示回退到分段位置控制。
    """
    # 选择可用的速度接口
    drive_fn = None
    if hasattr(chassis, "drive_speed"):
        drive_fn = getattr(chassis, "drive_speed")
    elif hasattr(chassis, "drive"):
        drive_fn = getattr(chassis, "drive")

    steps = max(1, int(duration / step))
    if drive_fn:
        cur_vx = cur_vy = cur_yaw = 0.0
        max_delta = max_accel * step
        for i in range(steps):
            cur_vx = _move_towards(cur_vx, vx, max_delta)
            cur_vy = _move_towards(cur_vy, vy, max_delta)
            cur_yaw = _move_towards(cur_yaw, yaw_rate, max_delta)
            try:
                # 尝试三参数调用，否则降级
                try:
                    drive_fn(cur_vx, cur_vy, cur_yaw)
                except TypeError:
                    drive_fn(cur_vx, cur_vy)
            except Exception:
                # 如果速度接口调用出错，停止并回退 (break loop)
                try:
                    drive_fn(0, 0, 0)
                except Exception:
                    pass
                return False
            time.sleep(step)
        # 结束时逐步减速到 0（平滑停）
        for i in range(steps):
            cur_vx = _move_towards(cur_vx, 0.0, max_delta)
            cur_vy = _move_towards(cur_vy, 0.0, max_delta)
            cur_yaw = _move_towards(cur_yaw, 0.0, max_delta)
            try:
                try:
                    drive_fn(cur_vx, cur_vy, cur_yaw)
                except TypeError:
                    drive_fn(cur_vx, cur_vy)
            except Exception:
                pass
            time.sleep(step)
        # 最终确保停下
        try:
            drive_fn(0, 0, 0)
        except Exception:
            try:
                drive_fn(0, 0)
            except Exception:
                pass
        return True
    else:
        # 回退：用小段位移平滑逼近（会比速度控制略差，但通常比一次大步更连贯）
        dx_per_step = vx * duration / steps
        dy_per_step = vy * duration / steps
        dz_per_step = yaw_rate * duration / steps  # 假设 z 以度为单位
        for i in range(steps):
            safe_move(chassis, x=dx_per_step, y=dy_per_step, z=dz_per_step, speed=None, wait=True, timeout=1)
            # very small sleep to avoid immediate stacking
            time.sleep(step)
        return False

def smooth_move_distance(chassis, x=0.0, y=0.0, z=0.0, total_time=1.0, max_accel=0.5, step=0.05):
    """
    当无法使用速度控制时，按小段分割位移以获得平滑效果。
    total_time 为总耗时（秒），会把 x/y/z 分为许多小步并顺序执行。
    """
    steps = max(1, int(total_time / step))
    dx = x / steps
    dy = y / steps
    dz = z / steps
    for i in range(steps):
        safe_move(chassis, x=dx, y=dy, z=dz, speed=None, wait=True, timeout=1)
        time.sleep(step)

if __name__ == '__main__':
    ep_robot = None
    try:
        # 不要在 Robot(...) 构造函数中传入 ip（新版本 SDK 不接受）
        ep_robot = robot.Robot()
        # 如需指定目标地址，请根据你当前 SDK 的 initialize 签名传入，例如：
        # ep_robot.initialize(target_addr="192.168.10.2")
        ep_robot.initialize()

        # 获取底盘控制对象
        chassis = ep_robot.chassis

        print("开始基础运动测试（平滑模式）...")

        # 使用速度控制（若支持），否则退回到分段距离控制
        # 示例：前进 0.5 m，保持 0.8s，总均速约 0.6 m/s
        if not smooth_move_velocity(chassis, vx=0.6, vy=0.0, yaw_rate=0.0, duration=0.8, max_accel=1.0, step=0.04):
            # 回退：按距离分段完成相同效果（0.5m）
            smooth_move_distance(chassis, x=0.5, y=0.0, z=0.0, total_time=0.8, step=0.04)

        # 后退
        if not smooth_move_velocity(chassis, vx=-0.6, vy=0.0, yaw_rate=0.0, duration=0.6, max_accel=1.0, step=0.04):
            smooth_move_distance(chassis, x=-0.3, y=0.0, z=0.0, total_time=0.6, step=0.04)

        # 侧移
        if not smooth_move_velocity(chassis, vx=0.0, vy=0.5, yaw_rate=0.0, duration=0.6, max_accel=1.0, step=0.04):
            smooth_move_distance(chassis, x=0.0, y=0.4, z=0.0, total_time=0.6, step=0.04)

        if not smooth_move_velocity(chassis, vx=0.0, vy=-0.4, yaw_rate=0.0, duration=0.5, max_accel=1.0, step=0.04):
            smooth_move_distance(chassis, x=0.0, y=-0.2, z=0.0, total_time=0.5, step=0.04)

        # 旋转：用速度控制更自然（yaw_rate 单位按 SDK 要求）
        if not smooth_move_velocity(chassis, vx=0.0, vy=0.0, yaw_rate=-90.0, duration=1.2, max_accel=180.0, step=0.04):
            smooth_move_distance(chassis, x=0.0, y=0.0, z=-90.0, total_time=1.2, step=0.04)

        if not smooth_move_velocity(chassis, vx=0.0, vy=0.0, yaw_rate=90.0, duration=1.2, max_accel=180.0, step=0.04):
            smooth_move_distance(chassis, x=0.0, y=0.0, z=90.0, total_time=1.2, step=0.04)

    except TypeError as e:
        print("初始化 Robot 时发生 TypeError:", e)
        raise
    finally:
        if ep_robot is not None:
            try:
                ep_robot.close()
            except Exception:
                pass
        print("基础运动测试结束（平滑模式）")