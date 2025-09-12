import time
from robomaster import robot
from robomaster import led

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_led = ep_robot.led

    print("你是谁？")#通过屏幕显示“你是谁？”

    while 1:#一直循环
        name=input("通过键盘输入你的名字：")#将键盘输入的文字存放在name里，以回车结束
        if name =="张鑫":#如果输入的名字是“张鑫”
            print("Hello!")#通过屏幕显示“Hello！”
            for count in range(5):
                ep_led.set_led(comp=led.COMP_ALL, r=0, g=255, b=0, effect=led.EFFECT_ON)
                time.sleep(0.2)
                ep_led.set_led(comp=led.COMP_ALL, r=0, g=255, b=0, effect=led.EFFECT_OFF)
                time.sleep(0.2)
        else:#否则
            print("我不认识你")#通过屏幕显示“我不认识你”
            for count in range(5):#重复5次
                ep_led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_ON)
                time.sleep(0.2)
                ep_led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_OFF)
                time.sleep(0.2)

    ep_robot.close()
