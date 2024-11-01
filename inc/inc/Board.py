#!/usr/bin/env python3
import os
import sys
import time
sys.path.append('home/rares/robot/')
import RPi.GPIO as GPIO
from smbus2 import SMBus, i2c_msg
from rpi_ws281x import PixelStrip
from rpi_ws281x import Color as PixelColor

if sys.version_info.major == 2:
    print('Please run this program with python3')
    sys.exit(0)

__ADC_BAT_ADRR = 0
__SERVO__ADDR = 21
__MOTOR_ADDR = 31
__SERVO_ADDR_CMD = 40

__motor_speed = [0, 0, 0, 0]
__servo_angle = [0, 0, 0, 0, 0, 0]
__servo_pulse = [0, 0, 0, 0, 0, 0]
__i2c = 1
__i2c_addr = 0x7A

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

    RGB_COUNT = 2
    RGB PIN = 12
    RGB FREQ HZ = 800000
    RGB DMA = 10
    RGB BRIGHTNESS = 120
    RGB CHANNEL = 0
    RGB_INVERT = False

RGB = PixelStrip(__RGB_COUNT,
_RGB_PIN, _RGB_FREQ_HZ, RGB_DMA, RGB_INVERT,
_RGB_BRIGHTNESS, RGB_CHANNEL)
RGB.begin() 
for i in range (RGB.numPixels()):
    RGB.setPixelColor(i, PixelColor(0,0,0))
    RGB.show()

def setMotor (index, speed):
    if index < 1 or index > 4:
        raise AttributeError("Invalid motor num: %d" %index)
    if index == 2 or index == 4:
        speed = speed
    else:
        speed = -speed
    index = index - 1
    speed = 100 if speed > 100 else speed
    speed = -100 if speed < -100 else speed
    reg = _MOTOR_ADDR + index

    with SMBus (__i2c) as bus:
        try:
        msg = i2c_msg.write(__i2c_addr, [reg, speed.to_bytes(1, 'little', signed=True)[0]])
        bus.i2c_rdwr(msg)
        _motor_speed[index] = speed
        except:
            msg = i2c_msg.write(___i2c_addr, [reg, speed.to_bytes(1, 'little', signed=True)[0]]) 
            bus.i2c_rdwr (msg)
            _motor_speed[index] = speed

    return _motor_speed[index]

def getMotor (index):
    if index < 1 or index > 4:
        raise AttributeError("Invalid motor num: %d" %index) 
    index = index - 1
    return _motor_speed[index]

def setPWMServoAngle (index, angle):
    if servo_id < 1 or servo_id> 6:
        raise AttributeError("Invalid Servo ID: %d" %servo_id)
    index = servo_id - 1
    angle = 180 if angle > 180 else angle
    angle = 0 if angle < else angle
    reg = SERVO_ADDR + index
    with SMBus (____i2c) as bus:
        try:
            msg = i2c_msg.write(__i2c_addr, [reg, angle]) 
            bus.i2c_rdwr(msg)
            _servo_angle [index] = angle
            _servo_pulse[index] = int(((200 * angle) / 9) + 500)

        except:
            msg = i2c_msg.write(__i2c_addr, [reg, angle]) 
            bus.i2c_rdwr(msg)
            servo_angle[index] = angle
            servo_pulse[index] = int (((200 * angle) / 9) + 500)

def setPWMServoPulse (servo_id, pulse = 1500, use_time = 1000):
    if servo_id< 1 or servo_id> 6:
        raise AttributeError("Invalid Servo ID: %d" %servo_id)
    index = servo_id - 1
    pulse = 500 if pulse < 500 else pulse
    pulse 2500 if pulse > 2500 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    buf = [_SERVO_ADDR_CMD, 1] + list (use_time.to_bytes (2, 'little')) + [servo_id,] + list(pulse.to_bytes (2, 'little'))

    with SMBus (__i2c) as bus:
        try:
            msg = i2c_msg.write(___i2c_addr, buf)
            bus.i2c_rdwr(msg)
            _servo_pulse[index] = pulse
            _servo_angle[index] = int((pulse 500) * 0.09)
        except BaseException as e:
            print(e)
            msg = i2c_msg.write(__i2c_addr, buf)
            bus.i2c_rdwr(msg)
            servo_pulse[index] = pulse
            _servo_angle [index] = int((pulse 500) * 0.09)

    return _servo_pulse[index]

def setPWMServosPulse (args):
    ''' time, number, idl, posl, id2, pos2... '''
    arglen = len(args) 
    servos = args[2:arglen:2]
    pulses = args [3:arglen:2]
    use_time = args[0]
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    servo_number = args[1]
    buf = [ _SERVO_ADDR_CMD, servo_number] + list(use_time.to_bytes (2, 'little'))
    dat= zip(servos, pulses)
    for (s, p) in dat:
        buf.append(s)
        p = 500 if p < 500 else p
        p = 2500 if p > 2500 else p
        buf += list(p.to_bytes (2, 'little'))
        servo_pulse [s-1] = p
        _servo_angle[s-1] = int((p - 500) * 0.09)

    with SMBus (__i2c) as bus:
        try:
            msg = i2c_msg.write(___i2c_addr, buf)
            bus.i2c_rdwr(msg)
        except:
            msg = i2c_msg.write(____i2c_addr, buf)
            bus.i2c_rdwr(msg)

def getPWMServoAngle(servo_id):
    if servo_id < 1 or servo_id> 6:
        raise AttributeError("Invalid Servo ID: %d" %servo_id)
        index = servo_id - 1
        return _servo_pulse[index]

def getBattery():
    ret = 0
    with SMBus (__i2c) as bus:
        try:
            msg = i2c_msg.write(__i2c_addr, [____ADC_BAT_ADDR, ])
            bus.i2c_rdwr(msg)
            read= i2c_msg.read(____i2c_addr, 2)
            bus.i2c_rdwr (read)
            ret = int.from_bytes (bytes (list(read)), 'little')
        except:
            msg = i2c_msg.write(____i2c_addr, [_ADC_BAT_ADDR, ])
            bus.i2c_rdwr(msg)
            read = i2c_msg.read(__i2c_addr, 2)
            bus.i2c_rdwr(read)
            ret = int.from_bytes (bytes (list(read)), 'little')

    return ret

def setBuzzer (new_state):
    GPIO.setup(31, GPIO.OUT)
    GPIO.output (31, new state)


def setBusServoID (oldid, newid):
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_ID_WRITE, newid)

def getBusServoID (id=None):
    while True:
        if id is None:
            serial_servo_read_cmd(0xfe, LOBOT_SERVO_ID_READ)
        else:
            serial_servo_read_cmd(id, LOBOT_SERVO_ID_READ)
        msg = serial_servo_get_rmsg (LOBOT_SERVO_ID_READ)
        if msg is not None:
            return msg


def setBusServoPulse (id, pulse, use_time):
    pulse = 0 if pulse < 0 else pulse
    pulse 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_TIME_WRITE, pulse, use_time)

def stopBusServo (id=None):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_STOP)

def setBusServoDeviation (id, d=0):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_ADJUST, d)    

def savеBusServоDeviation(la):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_WRITE)

time_out = 50

def getBusServoDeviation(id):
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_READ)
        msg = serial_servo_get_rmsg (LOBOT_SERVO_ANGLE_OFFSET_READ)
        count += 1
        if msg is not None:
            return msg
        if count > time_out:
            return None

def setBusServoAngleLimit (id, low, high):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_WRITE, low, high)

def getBusServoAngleLimit(id):
    while True:
    serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_READ)
    msg = serial_servo_get_rmsg (LOBOT_SERVO_ANGLE_LIMIT_READ)
    if msg is not None:
        count = 0
        return msg

def setBusServoVinLimit (id, low, high):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_VIN_LIMIT_WRITE, low, high)

def getBusServoVinLimit(id):

    while True:
    serial_servo_read_cmd(id, LOBOT_SERVO_VIN_LIMIT_READ)
    msg = serial_servo_get_rmsg (LOBOT_SERVO_VIN_LIMIT_READ)
    if msg is not None:
        return msg

def setBusServoMaxTemp (id, m_temp):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, m_temp)

def getBusServoTempLimit(id):
    while True:
    serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_READ) 
    msg = serial_servo_get_rmsg (LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        if msg is not None:
            return msg

def getBusServoPulse(id):
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_POS_READ)
        msg = serial_servo_get_rmsg (LOBOT_SERVO_POS_READ)
        if msg is not None:
            return msg

def getBusServoTemp(id):
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_READ)
        msg = serial_servo_get_rmsg (LOBOT_SERVO_TEMP_READ)
        if msg is not None:
            return msg

def getBusServoVin(id):
    while True:
    serial_servo_read_cmd(id, LOBOT_SERVO_VIN_READ)
    msg = serial_servo_get_rmsg (LOBOT_SERVO_VIN_READ)
    if msg is not None:
        return msg

def restBusServoPulse (oldid):
    serial_servo_set_deviation(oldid, 0)
    time.sleep(0.1)
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_MOVE_TIME_WRITE, 500, 100)

def unloadBusServo(id):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 0)

def getBusServoLoadStatus(id):
    while True:
    serial_servo_read_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
    msg = serial_servo_get_rmsg (LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
    if msg is not None:
        return msg
setBuzzer (0)        