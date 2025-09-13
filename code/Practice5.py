import cv2
import time
from robomaster import robot, vision

class MarkerInfo:
    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info

    @property
    def pt1(self):
        return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)

    @property
    def pt2(self):
        return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)

    @property
    def center(self):
        return int(self._x * 1280), int(self._y * 720)

    @property
    def text(self):
        return self._info


markers = []
current_distance = 0.0

def on_detect_marker(marker_info):
    number = len(marker_info)
    markers.clear()
    for i in range(0, number):
        x, y, w, h, info = marker_info[i]
        markers.append(MarkerInfo(x, y, w, h, info))

def on_distance_data(distance_info):
    global current_distance
    if distance_info:
        current_distance = distance_info[0] / 1000.0  # 毫米 → 米


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor

    ep_camera.start_video_stream(display=False)
    ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
    ep_sensor.sub_distance(freq=10, callback=on_distance_data)

    time.sleep(3)

    # 控制参数
    TARGET_CENTER = 0.5       # 图像中心点
    TARGET_DISTANCE = 0.5     # 目标距离 0.5m
    Kp_y = 0.6                # 横向平移比例系数
    Kp_x = 0.4                # 前后移动比例系数
    MAX_SPEED = 0.3           # 最大速度限制 (m/s)

    while True:
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        for j in range(0, len(markers)):
            cv2.rectangle(img, markers[j].pt1, markers[j].pt2, (255, 255, 255))
            cv2.putText(img, markers[j].text, markers[j].center,
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

        ############# P控制区域 ####################
        if markers:
            marker_x = markers[0]._x
            center_error = marker_x - TARGET_CENTER

            # 横向平移控制 (y方向)
            y_speed = Kp_y * center_error

            # 距离控制 (x方向) —— 已修正
            distance_error = current_distance - TARGET_DISTANCE
            if abs(distance_error) > 0.05 and current_distance > 0:
                x_speed = Kp_x * distance_error
            else:
                x_speed = 0

            # 限制最大速度，避免冲刺
            x_speed = max(min(x_speed, MAX_SPEED), -MAX_SPEED)
            y_speed = max(min(y_speed, MAX_SPEED), -MAX_SPEED)

            # 控制底盘：前进/后退 + 平移，不旋转
            ep_chassis.drive_speed(x=x_speed, y=y_speed, z=0, timeout=0.1)

        else:
            ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
        ############################################

        cv2.imshow("Markers", img)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    # 释放资源
    ep_vision.unsub_detect_info(name="marker")
    ep_sensor.unsub_distance()
    ep_camera.stop_video_stream()
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
    ep_robot.close()
    cv2.destroyAllWindows()
