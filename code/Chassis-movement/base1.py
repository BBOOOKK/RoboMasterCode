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

        print("开始基础运动测试...")

        safe_move(chassis, x=0.5, y=0, z=0, speed=0.3)
        time.sleep(1)

        safe_move(chassis, x=-0.3, y=0, z=0, speed=0.3)
        time.sleep(1)

        safe_move(chassis, x=0, y=0.4, z=0, speed=0.3)
        time.sleep(1)

        safe_move(chassis, x=0, y=-0.2, z=0, speed=0.3)
        time.sleep(1)

        safe_move(chassis, x=0, y=0, z=-90, speed=60)
        time.sleep(1)

        safe_move(chassis, x=0, y=0, z=90, speed=60)
        time.sleep(1)

    except TypeError as e:
        print("初始化 Robot 时发生 TypeError:", e)
        # 再抛出或根据需要处理
        raise
    finally:
        if ep_robot is not None:
            try:
                ep_robot.close()
            except Exception:
                # 忽略关闭时的异常，避免解释器退出时 __del__ 再次抛错
                pass
        print("基础运动测试结束")