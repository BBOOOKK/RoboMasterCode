from robomaster import robot
import importlib.util
import os
import time
import sys
import threading
import termios
import tty

# 动态加载 base2.py 中的控制函数（与 base2 保持一致的通信规则）
BASE2_PATH = os.path.join(os.path.dirname(__file__), "base2.py")
spec = importlib.util.spec_from_file_location("base2_module", BASE2_PATH)
base2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base2)

# 优先使用低延迟的轮速接口进行连续控制
_safe_drive_wheels = getattr(base2, "_safe_drive_wheels", None)
_safe_stop_wheels = getattr(base2, "_safe_stop_wheels", None)
s_move_wheels = getattr(base2, "s_move_wheels", None)
_clamp_wheel = getattr(base2, "_clamp_wheel", lambda v: int(max(-100, min(100, int(v)))))

def clamp_speed(v):
    try:
        v = float(v)
    except Exception:
        return 0.0
    return max(-100.0, min(100.0, v))

def help_text():
    return """操作说明（改进版）：
1) 先输入恒定速度（0~100），回车确认（作为基础力度）。
2) 进入键盘控制，按键行为（按一次开始持续，按同键停止；空格急停）：
   w：前进
   s：后退
   a：原地左转（in-place turn）
   d：原地右转（in-place turn）
   q：左平移（横移）
   e：右平移（横移）
   空格：急停 (stop)
   x：退出
   h：显示帮助
说明：终端无法捕获按键“松开”事件，本程序采用按键切换模式（按一次开始持续运动，再按一次停止），以实现连续与连贯控制。
"""

def getch_blocking():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def _move_towards_val(cur, target, max_delta):
    if abs(target - cur) <= max_delta:
        return target
    return cur + max_delta if target > cur else cur - max_delta

def map_cmd_to_wheels(cmd, speed):
    s = speed
    # 按照要求：a/d -> 原地转，q/e -> 平移
    if cmd == "w":
        return ( s,  s,  s,  s)
    if cmd == "s":
        return (-s, -s, -s, -s)
    if cmd == "a":  # 原地左转（counter-clockwise）
        return (-s, -s,  s,  s)
    if cmd == "d":  # 原地右转 (clockwise)
        return ( s,  s, -s, -s)
    if cmd == "q":  # 左移（横移）
        return (-s,  s, -s,  s)
    if cmd == "e":  # 右移（横移）
        return ( s, -s,  s, -s)
    return (0, 0, 0, 0)

def continuous_driver_loop(chassis, state):
    """
    持续驱动线程：按固定频率将当前轮速向目标轮速平滑过渡并下发。
    state: dict with keys: running(Event), target_cmd (str or None), base_speed (float),
           accel (float, units wheel_speed/second), tick (float seconds)
    """
    tick = state.get("tick", 0.04)
    accel = state.get("accel", 400.0)  # wheel speed units per second
    max_delta = accel * tick
    cur_w = [0.0, 0.0, 0.0, 0.0]
    try:
        while not state["running"].is_set():
            target = state.get("target_cmd", None)
            if target:
                tgt_w = map_cmd_to_wheels(target, state["base_speed"])
            else:
                tgt_w = (0.0, 0.0, 0.0, 0.0)
            # 平滑更新每个轮子的速度
            for i in range(4):
                cur_w[i] = _move_towards_val(cur_w[i], tgt_w[i], max_delta)
            # 下发指令（优先低延迟接口）
            try:
                if _safe_drive_wheels:
                    _safe_drive_wheels(chassis,
                                       _clamp_wheel(cur_w[0]),
                                       _clamp_wheel(cur_w[1]),
                                       _clamp_wheel(cur_w[2]),
                                       _clamp_wheel(cur_w[3]))
                elif s_move_wheels:
                    # 如果没有低延迟接口，降级为短时 S 曲线调用来模拟持续控制
                    s_move_wheels(chassis,
                                  int(cur_w[0]), int(cur_w[1]), int(cur_w[2]), int(cur_w[3]),
                                  duration=tick, step=min(0.02, tick/2), hold_ratio=0.7, peak_scale=1.0)
                else:
                    pass
            except Exception:
                # 忽略瞬时发送错误，下一轮会继续尝试
                pass
            time.sleep(tick)
    finally:
        # 退出时平滑停下
        try:
            # 逐步减速到 0
            for _ in range(6):
                for i in range(4):
                    cur_w[i] = _move_towards_val(cur_w[i], 0.0, max_delta)
                if _safe_drive_wheels:
                    _safe_drive_wheels(chassis,
                                       _clamp_wheel(cur_w[0]),
                                       _clamp_wheel(cur_w[1]),
                                       _clamp_wheel(cur_w[2]),
                                       _clamp_wheel(cur_w[3]))
                time.sleep(tick)
            if _safe_stop_wheels:
                _safe_stop_wheels(chassis)
        except Exception:
            try:
                if _safe_stop_wheels:
                    _safe_stop_wheels(chassis)
            except Exception:
                pass

def key_reader_loop(state):
    """
    读取按键并更新 state['target_cmd']：
    - 按动方向键（w/a/s/d/q/e）会切换该动作：按一次开始，按同键停止。
    - 空格停止，x 退出。
    """
    print(help_text())
    while not state["running"].is_set():
        ch = getch_blocking()
        if not ch:
            continue
        ch = ch.lower()
        if ch == "h":
            print(help_text())
            continue
        if ch == "x":
            # 退出
            state["running"].set()
            break
        if ch == " ":
            # 急停：取消目标
            state["target_cmd"] = None
            continue
        if ch in ("w", "a", "s", "d", "q", "e"):
            # 切换同命令状态（按一次开始，按再次停止）
            if state.get("target_cmd") == ch:
                state["target_cmd"] = None
            else:
                state["target_cmd"] = ch
            continue
        # 其他按键忽略

def main():
    ep_robot = None
    try:
        ep_robot = robot.Robot()
        ep_robot.initialize()
        chassis = ep_robot.chassis
        print("已连接 Robot。")
        # 输入恒定速度
        while True:
            try:
                base_speed = float(input("请输入恒定速度（0~100，推荐30~80）：").strip())
                if 0 < abs(base_speed) <= 100:
                    break
                else:
                    print("请输入 0~100 之间的数值。")
            except Exception:
                print("输入无效，请重新输入。")

        state = {
            "running": threading.Event(),
            "target_cmd": None,
            "base_speed": clamp_speed(base_speed),
            "accel": 600.0,  # 可调整的加速度（wheel speed units/s）
            "tick": 0.04
        }

        # 启动驱动线程
        driver_th = threading.Thread(target=continuous_driver_loop, args=(chassis, state), daemon=True)
        reader_th = threading.Thread(target=key_reader_loop, args=(state,), daemon=True)
        driver_th.start()
        reader_th.start()

        # 等待读取线程退出（按 x 退出）
        while not state["running"].is_set():
            time.sleep(0.1)

    except Exception as e:
        print("初始化/运行时异常:", e)
        raise
    finally:
        # 通知线程退出并等待
        try:
            if 'state' in locals():
                state["running"].set()
        except Exception:
            pass
        time.sleep(0.1)
        if ep_robot is not None:
            try:
                if _safe_stop_wheels:
                    _safe_stop_wheels(ep_robot.chassis)
            except Exception:
                pass
            try:
                ep_robot.close()
            except Exception:
                pass
        print("控制结束，已关闭连接。")

if __name__ == "__main__":
    main()