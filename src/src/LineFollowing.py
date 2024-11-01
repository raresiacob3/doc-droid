#coding=utf8
import sys
sys.path.append('/home/rares/robot/')
import cv2
import time
import math
import signal
import Camera
import threading
import numpy as np
import yaml_handle
import pandas as pd
import libraries.Board as Board
import libraries.mecanum as mecanum

if sys.version_info.major == 2:
    print("Please run this program with python3!")
    sys.exit(0)

servo1 = 1500
servo2 = 1500
target_color = {'red', 'green', 'blue'}

lab_data = None
servo_data = None

def load_config():
    global lab_data, servo_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

def initMove():
    Board.setPWMServoPulse(1, servo1, 1000)
    Board.setPWMServoPulse(2, servo2, 1000)

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255)
}

_stop = False
color_list = []
size = (640, 480)
_isRunning = False
detect_color = 'None'
start_pick_up = False
draw_color = range_rgb["black"]

def reset():
    global _stop, color_list, detect_color, start_pick_up, servo1, servo2
    _stop = False
    color_list = []
    detect_color = 'None'
    start_pick_up = False
    servo1 = servo_data['servo1']
    servo2 = servo_data['servo2']

def init():
    print("Line init")
    load_config()
    reset()
    initMove()

def start():
    global _isRunning
    reset()
    _isRunning = True
    print("Line start")

def stop():
    global _stop, _isRunning
    _stop = True
    _isRunning = False
    set_rgb('None')
    print("Line stop")

def exit():
    global _stop, _isRunning
    _stop = True
    _isRunning = False
    set_rgb('None')
    print("Line exit")

def setTargetColor(color):
    global target_color
    target_color = color
    return True

def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()

def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:
                area_max_contour = c

    return area_max_contour, contour_area_max

def move():
    global car_stop, _isRunning, detect_color

    while True:
        if _isRunning:
            if detect_color != 'red':
                set_rgb(detect_color)
            sensor_data = line.readData()

            if not sensor_data[0] and sensor_data[1] and sensor_data[2] and not sensor_data[3]:
                car.set_velocity(35, 90, 0)
                car_stop = True
            elif not sensor_data[0] and not sensor_data[1] and sensor_data[2] and not sensor_data[3]:
                car.set_velocity(35, 90, 0.03)
                car_stop = True
            elif not sensor_data[0] and sensor_data[1] and not sensor_data[2] and not sensor_data[3]:
                car.set_velocity(35, 90, -0.03)
                car_stop = True
            elif not sensor_data[0] and not sensor_data[1] and not sensor_data[2] and sensor_data[3]:
                car.set_velocity(35, 90, 0.3)
                car_stop = True
            elif not sensor_data[0] and not sensor_data[1] and not sensor_data[2] and not sensor_data[3]:
                car.set_velocity(35, 90, -0.3)
                car_stop = True
            elif not sensor_data[0] and sensor_data[1] and sensor_data[2] and sensor_data[3]:
                if car_stop:
                    car.set_velocity(0, 90, 0)
                    car_stop = False
                time.sleep(0.01)
                
            if detect_color == 'green':
                if not car_stop:
                    car.set_velocity(35, 90, 0)
                    car_stop = True

            else:
                if car_stop:
                    setBuzzer(0.1)
                    set_rgb(detect_color)
                    car.set_velocity(0, 90, 0)
                    car_stop = False
                    time.sleep(0.01)

        else:
            if car_stop:
                car.set_velocity(0, 90, 0)
                car_stop = False
                time.sleep(0.01)

th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

def run(img):
    global _isRunning, start_pick_up, detect_color, draw_color, color_list

    if not _isRunning:
        return img
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)

    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

    color_area_max = None
    max_area = 0
    areaMaxContour_max = 0
    if not start_pick_up:
        for i in target_color:
            if i in lab_data:
                frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0], lab_data[i]['min'][1], lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0], lab_data[i]['max'][1], lab_data[i]['max'][2])
                                         )
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = getAreaMaxContour(contours)
                if areaMaxContour is not None:
                    if area_max > max_area:
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour

    if max_area > 2500:
        rect = cv2.minAreaRect(areaMaxContour_max)
        box = np.int0(cv2.boxPoints(rect))

        cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)

    if not start_pick_up:
        if color_area_max == 'red':
            color = 1
        elif color_area_max == 'green':
            color = 2
        elif color_area_max == 'blue':
            color = 3
        else:
            color = 0
        color_list.append(color)

        if len(color_list) == 3:
            color = np.mean(np.array(color_list))
            color_list = []
            start_pick_up = True
            if color == 1:
                detect_color = 'red'
                draw_color = range_rgb["red"]
            elif color == 2:
                detect_color = 'green'
                draw_color = range_rgb["green"]
            elif color == 3:
                detect_color = 'blue'
                draw_color = range_rgb["blue"]
            else:
                start_pick_up = False
                detect_color = 'None'
                draw_color = range_rgb["black"]

    else:
        if not start_pick_up:
            detect_color = 'None'
            draw_color = range_rgb["black"]

    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)

    return img

def manual_stop(signum, frame):
    global _isRunning
    _isRunning = False
    car.set_velocity(0, 90, 0)

if __name__ == "__main__":
    init()
    start()
    camera = Camera.Camera()
    camera.camera_open(correction=True)
    signal.signal(signal.SIGINT, manual_stop)
    while _isRunning:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
