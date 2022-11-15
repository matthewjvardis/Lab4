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

def rad_to_deg(y):
    return(y*(180/math.pi))

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

def follow_wall(x, y, theta, lineTheta, slope, intercept, leftMotor, rightMotor, leftBump, rightBump, us, gyro, stop_watch):
    
    stop_watch.pause()
    stop_watch.reset()

    ideal = 300
    k = 0.5
    j = 0
    time_passed = 0

    while((time_passed < 100) or not(slope*x + intercept - 5 < y < slope*x + intercept + 5)):
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

x = 50
y = 0
gyro.reset_angle(90)
theta = gyro.angle()

slope = 4/3
intercept = -200/3
lineTheta = 180 + rad_to_deg(math.atan(-4/3))
#print(lineTheta)

stop_watch.resume()
run_motors(leftMotor, 180, rightMotor, 180)
wait(4000)
stop_motors(leftMotor, rightMotor)

stop_watch.pause()
t = stop_watch.time()/1000
x, y = update_position(x, y, theta, 180, 180, t)
print("x:", x)
print("y:", y)

while (not (177 < theta < 183)):
    stop_watch.resume()
    run_motors(leftMotor, -150, rightMotor, 150)
    wait(50)

    theta = gyro.angle()
    print(theta)

stop_motors(leftMotor, rightMotor)
stop_watch.pause()
stop_watch.reset()

fixedY = y
k = 1.5
while (not (204 < x)):

    stop_watch.resume()

    error = fixedY - y
    ur = -k * error + 180
    ul = k * error + 180

    run_motors(leftMotor, ul, rightMotor, ur)
    wait(50)

    stop_watch.pause()
    t = stop_watch.time()/1000
    stop_watch.reset()

    theta = gyro.angle()
    x, y = update_position(x, y, theta, ur, ul, t)
    print("x", x)
    print("y", y)

stop_motors(leftMotor, rightMotor)
wait(1000)

fixedX = x

while (not (87 < theta < 93)):
    run_motors(leftMotor, 150, rightMotor, -150)
    wait(50)

    theta = gyro.angle()
    print(theta)

stop_motors(leftMotor, rightMotor)
wait(1000)

while (not (225 < y)):

    stop_watch.resume()

    error = x - fixedX
    ur = -k * error + 180
    ul = k * error + 180

    run_motors(leftMotor, ul, rightMotor, ur)
    wait(100)

    stop_watch.pause()
    t = stop_watch.time()/1000
    stop_watch.reset()

    theta = gyro.angle()
    x, y = update_position(x, y, theta, ur, ul, t)
    print("x", x)
    print("y", y)

stop_motors(leftMotor, rightMotor)
ev3.speaker.play_file("Mariah.wav")

while (not (195 < x < 205 and 195 < y < 200)):
    stop_watch.resume()
    if (leftBump.pressed() or rightBump.pressed()):
        stop_watch.pause()
        stop_watch.reset()
        stop_motors(leftMotor, rightMotor)
        wait(1000)

        stop_watch.resume()
        run_motors(leftMotor, -150, rightMotor, -150)
        wait(2000)
        stop_motors(leftMotor, rightMotor)
        stop_watch.pause()
        t = stop_watch.time()/1000
        x, y = update_position(x, y, theta, -150, -150, t)
        stop_watch.reset()
        wait(1000)
        follow_wall(x, y, theta, lineTheta, slope, intercept, leftMotor, rightMotor, leftBump, rightBump, us, gyro, stop_watch)
    else:
        stop_watch.resume()
        run_motors(leftMotor, 150, rightMotor, 150)
        wait(50)
        stop_watch.pause()
        t = stop_watch.time()/1000
        stop_watch.reset()
        x, y = update_position(x, y, theta, 150, 150, t)
        print("x:", x)
        print("y:", y)
        print("theta:", )

stop_motors(leftMotor, rightMotor)

ev3.speaker.play_file("Mariah.wav")