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

def follow_wall(x, y, theta, leftMotor, rightMotor, leftBump, rightBump, us, gyro, stop_watch, goalX, goalY):
    
    stop_watch.pause()
    stop_watch.reset()


    run_motors(leftMotor, -150, rightMotor, 150)
    wait(2000)

    stop_motors(leftMotor, rightMotor)

    theta = gyro.angle()

    ideal = 200
    k = 0.5
    j = 0
    time_passed = 0

    while((time_passed < 100) or not(goalX - 5 < x < goalX + 5 or goalY < y < goalY + 5)):

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
    wait(200)
    ev3.speaker.beep()
    print("x:", x)
    print("y:", y)
    return x, y


# ------------ Initialization of Motors and Sensors ------------

ev3 = EV3Brick()

leftMotor = Motor(port = Port.C, positive_direction = Direction.COUNTERCLOCKWISE)
rightMotor = Motor(port = Port.B, positive_direction = Direction.COUNTERCLOCKWISE)

leftBump = TouchSensor(port = Port.S4)
rightBump = TouchSensor(port = Port.S1)

us = UltrasonicSensor(port = Port.S2)
gyro = GyroSensor(port = Port.S3)

speed = 250

wait_for_button()

stop_watch = StopWatch()
stop_watch.pause()
stop_watch.reset()

x = 50
y = 0
gyro.reset_angle(90)
theta = gyro.angle()

stop_watch.resume()
run_motors(leftMotor, speed, rightMotor, speed)
wait(3000)
stop_motors(leftMotor, rightMotor)

stop_watch.pause()
t = stop_watch.time()/1000
x, y = update_position(x, y, theta, speed, speed, t)
print("x:", x)
print("y:", y)

while (not (177 < theta < 183)):
    stop_watch.resume()
    run_motors(leftMotor, -150, rightMotor, 150)
    wait(25)

    theta = gyro.angle()
    print(theta)

stop_motors(leftMotor, rightMotor)
stop_watch.pause()
stop_watch.reset()

goalTheta = theta
k = 0.75

goalY = y

while (not (207 < x)):

    if (leftBump.pressed() or rightBump.pressed()):
        stop_motors(leftMotor, rightMotor)
        x, y = follow_wall(x, y, theta, leftMotor, rightMotor, leftBump, rightBump, us, gyro, stop_watch, 192, goalY)
        theta = gyro.angle()

        print("x:", x)
        print("y:", y)
        print("theta:", theta)

        while (not (177 < theta < 183)):
            run_motors(leftMotor, -150, rightMotor, 150)
            wait(25)
            theta = gyro.angle()

        stop_motors(leftMotor, rightMotor)
        wait(100)
        print("x:", x)
        print("y:", y)
        print("theta:", theta)
        stop_watch.pause()
        stop_watch.reset()
        continue

    stop_watch.resume()

    error = theta - 180
    ur = -k * error + speed
    ul = k * error + speed

    run_motors(leftMotor, ul, rightMotor, ur)
    wait(50)

    stop_watch.pause()
    t = stop_watch.time()/1000
    stop_watch.reset()

    theta = gyro.angle()
    x, y = update_position(x, y, theta, ur, ul, t)
    print("x:", x)
    print("y:", y)
    print("theta:", theta)

stop_motors(leftMotor, rightMotor)
wait(1000)

fixedX = x

while (not (87 < theta < 93)):
    run_motors(leftMotor, 150, rightMotor, -150)
    wait(25)

    theta = gyro.angle()
    print(theta)

stop_motors(leftMotor, rightMotor)
wait(1000)
goalTheta = theta

goalX = 200

while (not (230 < y)):
    
    if (leftBump.pressed() or rightBump.pressed()):
        stop_motors(leftMotor, rightMotor)
        x, y = follow_wall(x, y, theta, leftMotor, rightMotor, leftBump, rightBump, us, gyro, stop_watch, goalX, 0)
        theta = gyro.angle()

        print("x:", x)
        print("y:", y)
        print("theta:", theta)

        while (not (87 < theta < 93)):
            run_motors(leftMotor, -150, rightMotor, 150)
            wait(25)
            theta = gyro.angle()

        stop_motors(leftMotor, rightMotor)
        wait(100)
        print("x:", x)
        print("y:", y)
        print("theta:", theta)
        stop_watch.pause()
        stop_watch.reset()
        continue

    stop_watch.resume()

    error = theta - 90
    ur = -k * error + speed
    ul = k * error + speed

    run_motors(leftMotor, ul, rightMotor, ur)
    wait(100)

    stop_watch.pause()
    t = stop_watch.time()/1000
    stop_watch.reset()

    theta = gyro.angle()
    x, y = update_position(x, y, theta, ur, ul, t)
    print("x", x)
    print("y", y)

stop_watch.pause()
stop_watch.reset()

stop_motors(leftMotor, rightMotor)
ev3.speaker.play_file("Mariah.wav")