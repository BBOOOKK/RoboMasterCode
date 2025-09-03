from robomaster import robot
import time

def _safe_drive_wheels(chassis, w1, w2, w3, w4):
    """
    兼容不同 SDK 的轮速接口调用：
    - 优先尝试 chassis.drive_wheels(...)（关键字或位置参数）；
    - 其次尝试常见的同类替代名（set_wheel_speed(s) 等）。
    返回 True 表示调用成功，False 表示没有找到可用接口或调用失败。
    """
    # 常见接口名称及可能签名
    if hasattr(chassis, "drive_wheels"):
        fn = getattr(chassis, "drive_wheels")
        try:
            # 先尝试关键字参数
            fn(w1=w1, w2=w2, w3=w3, w4=w4)
            return True
        except TypeError:
            try:
                # 再尝试位置参数
                fn(w1, w2, w3, w4)
                return True
            except Exception:
                return False
        except Exception:
            return False
    for name in ("set_wheel_speed", "set_wheel_speeds", "set_wheels_speed"):
        if hasattr(chassis, name):
            fn = getattr(chassis, name)
            try:
                fn(w1=w1, w2=w2, w3=w3, w4=w4)
                return True
            except TypeError:
                try:
                    fn(w1, w2, w3, w4)
                    return True
                except Exception:
                    return False
            except Exception:
                return False
    return False

def _safe_stop_wheels(chassis):
    # 尽量保证调用停止命令，忽略异常
    try:
        _safe_drive_wheels(chassis, 0, 0, 0, 0)
    except Exception:
        try:
            if hasattr(chassis, "stop"):
                chassis.stop()
        except Exception:
            pass

# ---------- 新增：S 型速度包络与连续运动控制 ----------
def _smoothstep(x):
    """标准 smoothstep (S 型) 3x^2 - 2x^3，输入 0..1 -> 输出 0..1"""
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    return x * x * (3 - 2 * x)

def _clamp_wheel(v):
    """将轮速限制到典型范围，-100..100（根据硬件/SDK调整）"""
    try:
        v = int(v)
    except Exception:
        v = 0
    if v > 100:
        return 100
    if v < -100:
        return -100
    return v

def s_move_wheels(chassis, w1, w2, w3, w4, duration=1.0, step=0.04, hold_ratio=0.6, peak_scale=1.0):
    """
    用 S 曲线包络从 0 平滑过渡到目标轮速并回落到 0。
    - duration: 总时间（秒）
    - step: 采样间隔（秒），越小越平滑但命令更多
    - hold_ratio: 中间占比保持最大速度，其余两边均分为加/减速段（0..1）
    - peak_scale: 目标轮速缩放倍率（1.0 = 原目标）
    最终会调用停止保证停稳。
    """
    steps = max(2, int(duration / step))
    # clamp hold_ratio
    hold_ratio = max(0.0, min(0.98, hold_ratio))
    ramp_ratio = max(0.0, (1.0 - hold_ratio) / 2.0)

    for i in range(steps):
        t = i / (steps - 1)
        if t < ramp_ratio and ramp_ratio > 0:
            s = t / ramp_ratio
            env = _smoothstep(s)
        elif t > 1.0 - ramp_ratio and ramp_ratio > 0:
            s = (1.0 - t) / ramp_ratio
            env = _smoothstep(s)
        else:
            env = 1.0
        scale = env * peak_scale
        _safe_drive_wheels(chassis,
                           _clamp_wheel(w1 * scale),
                           _clamp_wheel(w2 * scale),
                           _clamp_wheel(w3 * scale),
                           _clamp_wheel(w4 * scale))
        time.sleep(step)
    # 确保平滑停下
    try:
        _safe_drive_wheels(chassis, 0, 0, 0, 0)
    except Exception:
        try:
            if hasattr(chassis, "stop"):
                chassis.stop()
        except Exception:
            pass

# ---------- end 新增 ----------

if __name__ == '__main__':
    ep_robot = None
    try:
        # 遵循 base1 的通信/初始化规则：不要把 ip 直接传给 Robot()
        ep_robot = robot.Robot()
        ep_robot.initialize()

        chassis = ep_robot.chassis

        print("开始连续运动测试（S 型包络）...")

        # 1. 左右轮同速前进（提升到 80% 并更快完成）
        s_move_wheels(chassis, 80, 80, 80, 80, duration=1.6, step=0.03, hold_ratio=0.5, peak_scale=1.0)

        # 2. 减速前进（较低速度，短时）
        s_move_wheels(chassis, 45, 45, 45, 45, duration=1.0, step=0.03, hold_ratio=0.4)

        # 3. 原地顺时针旋转（左轮正转，右轮反转）
        s_move_wheels(chassis, 60, 60, -60, -60, duration=1.2, step=0.03, hold_ratio=0.5)

        # 4. 向右平移（对角线轮子同速，方向相反）
        s_move_wheels(chassis, 60, -60, 60, -60, duration=1.2, step=0.03, hold_ratio=0.5)

        # 5. 停止所有运动（冗余确保）
        _safe_stop_wheels(chassis)

        print("连续运动测试结束（S 型包络）")

    except Exception as e:
        print("运行时发生异常:", e)
        raise
    finally:
        if ep_robot is not None:
            try:
                _safe_stop_wheels(ep_robot.chassis)
            except Exception:
                pass
            try:
                ep_robot.close()
            except Exception:
                pass