# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time
import sys
import traceback
import importlib.util
import os

from robomaster import robot, led

HERE = os.path.dirname(__file__)

def try_init(conn_type, target=None, timeout=5.0):
    ep = robot.Robot()
    try:
        print(f"尝试初始化 conn_type={conn_type} target={target}")
        if target is not None:
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
        traceback.print_exc(limit=3)
        try:
            ep.close()
        except Exception:
            pass
        return None

def _set_led_best_effort(ep_led, comp, r, g, b, effect=None):
    if ep_led is None:
        return False
    try:
        # 常见接口：set_led(name/index, r,g,b[, effect])
        if hasattr(ep_led, "set_led"):
            try:
                # 一些版本接受 (name, r,g,b)
                ep_led.set_led(comp, r, g, b) if effect is None else ep_led.set_led(comp, r, g, b, effect)
                return True
            except Exception:
                # 有些实现需要 index 而不是 name
                try:
                    idx_map = {"front":0,"left":1,"right":2,"back":3,"all":0}
                    idx = idx_map.get(comp, comp)
                    ep_led.set_led(idx, r, g, b)
                    return True
                except Exception:
                    pass
    except Exception:
        pass
    try:
        # 有些实现提供 set_all / set_leds
        if hasattr(ep_led, "set_all"):
            ep_led.set_all(r, g, b)
            return True
        if hasattr(ep_led, "set_leds"):
            # try mapping by dict
            try:
                ep_led.set_leds({"front":(r,g,b),"left":(0,0,0),"right":(0,0,0),"back":(0,0,0)})
                return True
            except Exception:
                pass
    except Exception:
        pass
    return False

def _clear_all_chassis(ep_led):
    try:
        _set_led_best_effort(ep_led, "front", 0,0,0)
        _set_led_best_effort(ep_led, "left", 0,0,0)
        _set_led_best_effort(ep_led, "right", 0,0,0)
        _set_led_best_effort(ep_led, "back", 0,0,0)
    except Exception:
        pass

def _apply_pattern(ep_led, pattern_on):
    """
    pattern_on: dict with keys front,left,right,back -> (r,g,b) or None
    """
    if ep_led is None:
        return
    for comp, rgb in pattern_on.items():
        if rgb is None:
            _set_led_best_effort(ep_led, comp, 0,0,0)
        else:
            r,g,b = rgb
            _set_led_best_effort(ep_led, comp, r, g, b)

# 预定义模式（满足用户要求）
_PATTERNS = {
    "forward": {"front":(255,255,255), "left":(0,0,0), "right":(0,0,0), "back":(0,0,0)},
    "backward": {"front":(255,255,255), "left":(0,0,0), "right":(0,0,0), "back":(255,255,255)},
    "left_turn": {"front":(255,255,255), "left":(255,170,0), "right":(0,0,0), "back":(0,0,0)},
    "right_turn": {"front":(255,255,255), "left":(0,0,0), "right":(255,170,0), "back":(0,0,0)},
}

def blink_chassis(ep_led, mode="forward", blink_side=False, interval=0.3, duration=5.0):
    """
    让底盘灯按 mode 闪烁。
    - mode: forward/backward/left_turn/right_turn
    - blink_side: 对于 left_turn/right_turn 是否让侧灯周期性闪烁
    - interval: 闪烁间隔（秒）
    - duration: 总时长（秒），<=0 表示一直闪到 KeyboardInterrupt
    """
    start = time.time()
    ep = ep_led
    base_pattern = _PATTERNS.get(mode, _PATTERNS["forward"])
    side = "left" if mode == "left_turn" else ("right" if mode == "right_turn" else None)

    try:
        while True:
            # 开
            _apply_pattern(ep, base_pattern if not side or not blink_side else {
                "front": base_pattern["front"],
                "left": base_pattern["left"] if side!="left" else base_pattern["left"],
                "right": base_pattern["right"] if side!="right" else base_pattern["right"],
                "back": base_pattern["back"],
            })
            # 当是转向且需要闪烁时，侧灯单独控制为亮
            if side and blink_side:
                # 点亮侧灯
                _set_led_best_effort(ep, side, 255,170,0)
                # 关闭另一侧确保一致
                other = "left" if side=="right" else "right"
                _set_led_best_effort(ep, other, 0,0,0)
            time.sleep(interval)
            # 关（侧灯或所有灯）
            if side and blink_side:
                # 侧灯熄灭，前灯保持白（或全部熄灭以突出闪烁效果）
                _set_led_best_effort(ep, side, 0,0,0)
                _set_led_best_effort(ep, "front", 255,255,255)
            else:
                # 非转向或不闪烁时，整体清除一小段时间以实现“闪烁感”
                _clear_all_chassis(ep)
            time.sleep(interval)
            if duration > 0 and (time.time() - start) >= duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        _clear_all_chassis(ep)

def main():
    ep_robot = None
    # 先尝试 ap，再 sta
    ep_robot = try_init("ap")
    if ep_robot is None:
        ep_robot = try_init("sta")
    if ep_robot is None:
        print("无法连接机器人（ap/sta 均失败）。请检查 Wi‑Fi 与 robomaster SDK 版本。")
        sys.exit(1)

    try:
        ep_led = ep_robot.led
        # 示例：按四种模式循环，每种模式闪烁 4 秒，侧灯在转向时闪烁
        modes = [("forward", False), ("backward", False), ("left_turn", True), ("right_turn", True)]
        for mode, blink_side in modes:
            print("模式:", mode, "blink_side=", blink_side)
            blink_chassis(ep_led, mode=mode, blink_side=blink_side, interval=0.25, duration=4.0)
    finally:
        try:
            _clear_all_chassis(ep_robot.led)
        except Exception:
            pass
        try:
            ep_robot.close()
        except Exception:
            pass

if __name__ == '__main__':
    main()
#

