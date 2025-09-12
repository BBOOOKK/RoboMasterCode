import time
import sys
import traceback
import importlib.util
import os

import robomaster
from robomaster import robot, led

HERE = os.path.dirname(__file__)

def _load_helper(name):
    path = os.path.join(HERE, "..", "Chassis-movement", name)
    if not os.path.exists(path):
        return None
    spec = importlib.util.spec_from_file_location(name[:-3], path)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
        return mod
    except Exception:
        return None

# 尝试加载 base1 / base2（如果有自定义连接辅助，可调用）
base1 = _load_helper("base1.py")
base2 = _load_helper("base2.py")

def try_init(conn_type, target=None, timeout=5.0):
    ep = robot.Robot()
    try:
        print(f"尝试初始化 conn_type={conn_type} target={target}")
        if target is not None:
            # 有些 SDK 版本使用 host/target 参数不同，尝试常见调用方式
            try:
                ep.initialize(conn_type=conn_type, target=target)
            except TypeError:
                ep.initialize(conn_type=conn_type, host=target)
        else:
            ep.initialize(conn_type=conn_type)
        print("初始化成功:", conn_type)
        return ep
    except Exception as e:
        print(f"初始化失败（conn_type={conn_type}, target={target}）: {e}")
        traceback.print_exc(limit=5)
        try:
            ep.close()
        except Exception:
            pass
        return None

def _set_gimbal_led_best_effort(ep_led, comp, r, g, b, led_list=None, effect=None):
    """多方式尝试设置云台 LED，尽量兼容不同 SDK 版本"""
    if ep_led is None:
        return False
    # 优先尝试官方接口
    try:
        if hasattr(ep_led, "set_gimbal_led"):
            kwargs = {}
            if led_list is not None:
                kwargs["led_list"] = led_list
            if effect is not None:
                kwargs["effect"] = effect
            ep_led.set_gimbal_led(comp=comp, r=r, g=g, b=b, **kwargs)
            return True
    except Exception:
        pass
    # 退回到 set_led / set_leds 等通用接口（按索引或区域尝试）
    try:
        if hasattr(ep_led, "set_led"):
            # 有些实现：set_led(index, r,g,b) 或 set_led(name, r,g,b)
            try:
                if isinstance(comp, str):
                    ep_led.set_led(comp, r, g, b)
                else:
                    ep_led.set_led(comp, r, g, b)
                return True
            except Exception:
                pass
    except Exception:
        pass
    # 最后尝试顶层 robot.set_led（若 available）
    try:
        robot_obj = getattr(ep_led, "_robot_ref", None)
        if robot_obj is None:
            # no reference: try nothing more
            pass
    except Exception:
        pass
    return False

def main():
    ep_robot = None
    # 优先使用 base1/base2 中可能提供的连接 helper（若存在）
    # 常见 pattern: baseX 模块可能提供 connect / init helpers — 若存在可尝试调用
    connect_tried = False
    if base1 and hasattr(base1, "connect_robot"):
        try:
            print("使用 base1.connect_robot 连接")
            ep_robot = base1.connect_robot()
            connect_tried = True
        except Exception:
            ep_robot = None
    if ep_robot is None and base2 and hasattr(base2, "connect_robot"):
        try:
            print("使用 base2.connect_robot 连接")
            ep_robot = base2.connect_robot()
            connect_tried = True
        except Exception:
            ep_robot = None

    # 常规回退：先 ap，再 sta
    if ep_robot is None:
        ep_robot = try_init("ap")
    if ep_robot is None:
        ep_robot = try_init("sta")

    if ep_robot is None:
        print("无法连接机器人（尝试 base1/base2 helper 与 ap/sta 均失败）。")
        print("请检查：Wi‑Fi（是否连接 robot AP 或在同一局域网）、robomaster SDK 版本、权限。")
        sys.exit(1)

    try:
        ep_led = ep_robot.led
        for i in range(0, 8):
            led1 = i % 8
            led2 = (i + 1) % 8
            led3 = (i + 2) % 8
            ok = _set_gimbal_led_best_effort(ep_led, comp=led.COMP_TOP_ALL, r=255, g=25, b=25,
                                             led_list=[led1, led2, led3], effect=getattr(led, "EFFECT_ON", None))
            print("Gimbal Led set:", led1, led2, led3, "ok=" + str(bool(ok)))
            time.sleep(0.5)
    finally:
        try:
            ep_robot.close()
        except Exception:
            pass
if __name__ == '__main__':
    main()