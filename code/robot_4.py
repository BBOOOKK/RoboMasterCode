import robomaster
from robomaster import robot
from robomaster import led
import time

distance=[]

def sub_data_handler(sub_info):
    global distance
    distance = sub_info
    #print("tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(distance[0], distance[1], distance[2], distance[3]))

if __name__ == '__main__':
    ep_robot = robot.Robot()
    
    ep_robot.initialize(conn_type="ap")

    ep_sensor = ep_robot.sensor
    ep_sensor.sub_distance(freq=5, callback=sub_data_handler)
    ep_chassis = ep_robot.chassis
    ep_led = ep_robot.led
    time.sleep(3)
    while True:
        if distance[0]==0:
            ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
            ep_led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_ON)
        else:
            print(distance[0])
            ep_led.set_led(comp=led.COMP_ALL, r=0, g=0, b=255, effect=led.EFFECT_ON)
            if distance[0]>400:#静止区
                ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
            elif 400>=distance[0]>250:#跟进区
                ep_chassis.drive_speed(x=0.1, y=0, z=0, timeout=5)
            elif 250>=distance[0]>150:#稳定区
                ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                ep_led.set_led(comp=led.COMP_ALL, r=255, g=0, b=255, effect=led.EFFECT_ON)
            elif 150>=distance[0]>1:#后退区
                ep_chassis.drive_speed(x=-0.1, y=0, z=0, timeout=5)
            else:
                ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
        time.sleep(0.1)
    ep_sensor.unsub_distance()
    ep_robot.close()