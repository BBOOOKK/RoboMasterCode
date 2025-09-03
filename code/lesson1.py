import time
from robomaster import robot
from robomaster import led


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type = "ap")

    ep_led = ep_robot.led

    print("who are u ?")

    while 1:
        name = input ("Please input: ")
        if name == "book" :
            print("HELLO!")
            for count in range(5):
                ep_led.set_led(comp = led.COMP_ALL, r=0, g=255, b=0, effect = led.EFFECT_ON)
                time.sleep(0.2)
                ep_led.set_led(comp = led.COMP_ALL, r=0, g=255, b=0, effect = led.EFFECT_OFF)
                time.sleep(0.2)
        else : 
            print("?")
            for count in range(5):
                ep_led.set_led(comp = led.COMP_ALL, r=255, g=0, b=0, effect = led.EFFECT_ON)
                time.sleep(0.2)
                ep_led.set_led(comp = led.COMP_ALL, r=255, g=0, b=0, effect = led.EFFECT_OFF)
                time.sleep(0.2)

    ep_robot.close()

