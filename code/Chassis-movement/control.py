from robomaster import robot
import importlib.util
import os
import time
import sys

# 动态加载 base2.py 中的 s_move_wheels/_safe_stop_wheels（与 base2 保持一致的通信规则）
BASE2_PATH = os.path.join(os.path.dirname(__file__), "base2.py")
spec = importlib.util.spec_from_file_location("base2_module", BASE2_PATH)
base2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base2)

s_move_wheels = getattr(base2, "s_move_wheels")
_safe_stop_wheels = getattr(base2, "_safe_stop_wheels")

def clamp_speed(v):
    try:
        v = float(v)
    except Exception:
        return 0.0
    return max(-100.0, min(100.0, v))

def help_text():
    return """命令格式（空格分隔）：
  f <speed> <duration>    前进（speed 0..100，duration 秒）
  b <speed> <duration>    后退
  r <speed> <duration>    右平移（横移）
  l <speed> <duration>    左平移
  rc <speed> <duration>   原地顺时针旋转
  rcc <speed> <duration>  原地逆时针旋转
  stop                    立即停止
  q / quit                退出
示例：f 60 1.2   （以 60 的强度前进 1.2 秒）
"""

def cmd_loop(chassis):
    print(help_text())
    while True:
        try:
            line = input("cmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n退出控制。")
            break
        if not line:
            continue
        toks = line.split()
        cmd = toks[0].lower()
        if cmd in ("q", "quit"):
            break
        if cmd == "stop":
            _safe_stop_wheels(chassis)
            continue
        if cmd in ("f", "b", "r", "l", "rc", "rcc"):
            if len(toks) < 3:
                print("参数不足。示例：f 60 1.0")
                continue
            speed = clamp_speed(toks[1])
            dur = 1.0
            try:
                dur = float(toks[2])
            except Exception:
                pass
            # 根据 base2 的 wheel 签名约定构造轮速（w1,w2,w3,w4）
            # base2 里：前进 -> (s,s,s,s) ; 右平移 -> (s,-s,s,-s) ; 旋转顺时针 -> (s,s,-s,-s)
            if cmd == "f":
                w = ( speed,  speed,  speed,  speed)
            elif cmd == "b":
                w = (-speed, -speed, -speed, -speed)
            elif cmd == "r":
                w = ( speed, -speed,  speed, -speed)
            elif cmd == "l":
                w = (-speed,  speed, -speed,  speed)
            elif cmd == "rc":
                w = ( speed,  speed, -speed, -speed)
            elif cmd == "rcc":
                w = (-speed, -speed,  speed,  speed)
            else:
                w = (0,0,0,0)
            # 使用 S 曲线包络执行（短时间内可连续调用以获得持续控制）
            try:
                s_move_wheels(chassis, int(w[0]), int(w[1]), int(w[2]), int(w[3]),
                              duration=max(0.02, dur), step=0.03, hold_ratio=0.6, peak_scale=1.0)
            except Exception as e:
                print("执行移动出错:", e)
            continue
        print("未知命令。输入空行或 Ctrl+C 退出，或输入 help 查看帮助。")

def main():
    ep_robot = None
    try:
        ep_robot = robot.Robot()
        ep_robot.initialize()
        chassis = ep_robot.chassis
        print("已连接 Robot，进入终端控制模式（输入 q 退出）。")
        cmd_loop(chassis)
    except Exception as e:
        print("初始化/运行时异常:", e)
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
        print("控制结束，已关闭连接。")

if __name__ == "__main__":
    main()