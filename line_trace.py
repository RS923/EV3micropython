#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

maxspeed = 400
value = 30
kp = 1.8
ki = 0
kd = 0.5
DELTA_T = 0.005
diff = [0, 0]
integral = 0
def power(percentage):
  return maxspeed*percentage/100
def math_limit(x, y=100, z=-100):
  if x>y:
    return y
  if x<z:
    return z
  return x
mR = Motor(Port.A, Direction.COUNTERCLOCKWISE)
mL = Motor(Port.B, Direction.COUNTERCLOCKWISE)
tL = TouchSensor(Port.S3)
tR = TouchSensor(Port.S4)
cR = ColorSensor(Port.S1)
cL = ColorSensor(Port.S2)
while tL.pressed() == 0:
  wait(5)
while True:
  valueR = cR.reflection()
  valueL = cL.reflection()
  diff[0] = diff[1]
  diff[1] = valueR-valueL
  integral += (diff[1]+diff[0])/2.0*DELTA_T
  p = kp*diff[1]
  i = ki*integral
  d = kd*(diff[1]-diff[0])/DELTA_T
  pid = math_limit(p+i+d)
  powerR = 80+(valueR-valueL)*kp
  powerL = 80-(valueR-valueL)*kp
  mR.run(power(powerR))
  mL.run(power(powerL))
  if valueR <= 20 and valueL <=20: #for later use
    brick.sound.beep()
