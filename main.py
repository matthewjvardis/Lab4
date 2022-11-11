#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import math


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

def deg_to_rad(y):
    return(y*(math.pi/180))

def run_motors(leftMotor, leftSpeed, rightMotor, rightSpeed):
    leftMotor.run(-leftSpeed)
    rightMotor.run(-rightSpeed)

def stop_motors(leftMotor, rightMotor):
    while (leftMotor.speed() != 0 or rightMotor.speed() != 0):
        leftMotor.run_time(0,0,then=Stop.BRAKE, wait = False)
        rightMotor.run_time(0,0,then=Stop.BRAKE, wait = False)

def wait_for_button():
    pressed = 0
    while (Button.CENTER not in EV3Brick.buttons.pressed()):
        pass

def update_position(x, y, theta, ur, ul, t, l=14.9, r=3):

    ur = -deg_to_rad(ur)
    ul = -deg_to_rad(ul)

    theta = deg_to_rad(-theta)

    vr = ur * r
    vl = ul * r

    #theta = -theta

    if (vr - vl == 0):
        newx = x + vr * math.cos(theta)*t
        newy = y + vl * math.sin(theta)*t
        return newx, newy

    radius = (l/2) * (vr+vl)/(vr-vl)
    iccx = x - radius*math.sin(theta)
    iccy = y + radius*math.cos(theta)
    omega = (vr-vl)/l
    alpha = omega * t

    newx = math.cos(alpha)*(x-iccx) - math.sin(alpha)*(y-iccy) + iccx
    newy = math.sin(alpha)*(x-iccx) + math.cos(alpha)*(y-iccy) + iccy
    return newx, newy

# ------------ Initialization of Motors and Sensors ------------

ev3 = EV3Brick()

leftMotor = Motor(port = Port.C, positive_direction = Direction.COUNTERCLOCKWISE)
rightMotor = Motor(port = Port.B, positive_direction = Direction.COUNTERCLOCKWISE)

leftBump = TouchSensor(port = Port.S4)
rightBump = TouchSensor(port = Port.S2)

us = UltrasonicSensor(port = Port.S1)
gyro = GyroSensor(port = Port.S3)

wait_for_button()

stop_watch = StopWatch()
stop_watch.pause()
stop_watch.reset()
stop_watch.resume()

x = 50
y = 0
gyro.reset_angle(90)
theta = gyro.angle()


# ------------ Following a straight line until it reaches an obstacle ------------

while (not (leftBump.pressed() or rightBump.pressed())):
    run_motors(leftMotor, 180, rightMotor, 180)

stop_watch.pause()
t = stop_watch.time() / 1000
stop_watch.reset()

stop_motors(leftMotor, rightMotor)
theta = gyro.angle()
print(theta)

x, y = update_position(x, y, theta, 180, 180, t)

run_motors(leftMotor, -150, rightMotor, 150)
wait(2500)

stop_motors(leftMotor, rightMotor)
theta = gyro.angle()

x, y = update_position(x, y, theta, -150, 150, 2.5)
ev3.speaker.beep()

wait(1500)

# ------------ Following obstacle until m-line is reached ------------

ideal = 300
k = 0.5
j = 0

time_passed = 0

while((time_passed < 300) or not (45 < x < 55 and -30 < y < 30)):

    stop_watch.resume()

    # Edge case: if robot bumps wall, turn robot away from wall and resume
    if(leftBump.pressed() or rightBump.pressed()):
        stop_watch.pause()
        stop_watch.reset()

        stop_motors(leftMotor, rightMotor)

        run_motors(leftMotor, -80, rightMotor, 200)
        wait(2000)

        stop_motors(leftMotor, rightMotor)
        x, y = update_position(x, y, theta, 200, -80, 2)
        
        print("x:", str(x))
        print("y:", str(y))
        continue

     # Proportional controller

    dist = us.distance()

    error = dist-ideal
    error = min(error, 80)

    ur = -k * error + 150
    ul = k * error + 150
    run_motors(leftMotor, ul, rightMotor, ur)

    wait(100)

    stop_watch.pause()
    t = stop_watch.time() / 1000

    # Calculating new robot position
    theta = gyro.angle()
    x, y = update_position(x, y, theta, ul, ur, t)

    if (j % 5 == 0):
        print("x:", str(x))
        print("y:", str(y))
        print("theta:", str(theta))

    j += 1
    time_passed += 1
    stop_watch.reset()

stop_motors(leftMotor, rightMotor)
ev3.speaker.beep()
stop_watch.pause()
stop_watch.reset()

run_motors(leftMotor, 100, rightMotor, -100)
wait(2000)

stop_watch.reset()
stop_motors(leftMotor, rightMotor)

ev3.speaker.play_file("Mariah.wav")