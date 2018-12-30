#-*- coding: utf-8 -*-
'''
Created on 30 avr. 2018

@author: PASTOR Robert

Python code example of converting Roll Pitch Yaw / Euler angles to Rotation Vector/Angle Axis for Universal-Robots.

Python code example of converting RPY/Euler angles to Rotation Vector/Angle Axis for Universal-Robots.
Original post by Erwin Damsma.

Mind you that when all input values are 0, there will be a division by zero error at this line:
# multi = 1 / (2 * math.sin(theta))
In this case theta will be 0 and the sin of 0 is 0, so you will probably have to make a special case for this and set the rx, ry and rz to 0 directly.


'''

import math
import numpy as np

roll = 2.6335
pitch = 0.4506
yaw = 1.1684

print "roll = ", roll
print "pitch = ", pitch
print "yaw = ", yaw
print ""

def compute(yaw, pitch, roll):

    yawMatrix = np.matrix([
    [math.cos(yaw), -math.sin(yaw), 0],
    [math.sin(yaw), math.cos(yaw), 0],
    [0, 0, 1]
    ])
    
    pitchMatrix = np.matrix([
    [math.cos(pitch), 0, math.sin(pitch)],
    [0, 1, 0],
    [-math.sin(pitch), 0, math.cos(pitch)]
    ])
    
    rollMatrix = np.matrix([
    [1, 0, 0],
    [0, math.cos(roll), -math.sin(roll)],
    [0, math.sin(roll), math.cos(roll)]
    ])
    
    R = yawMatrix * pitchMatrix * rollMatrix
    
    theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
    rx=0.0
    ry=0.0
    rz=0.0
    if (theta>0.0):
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (R[2, 1] - R[1, 2]) * theta
        ry = multi * (R[0, 2] - R[2, 0]) * theta
        rz = multi * (R[1, 0] - R[0, 1]) * theta
    
    return rx,ry,rz

rx,ry,rz = compute(yaw, pitch, roll)
print rx, ry, rz
print '---------------------------'
rx,ry,rz = compute(0.0, 0.0, 0.0)
print rx, ry, rz
print '---------------------------'
rx,ry,rz = compute(0.0, math.radians(180.0), math.radians(20.0))
print rx, ry, rz
print math.degrees(ry)
print math.degrees(rz)
print '---------------------------'
rx,ry,rz = compute(math.radians(20.0),  math.radians(180.0), 0.0)
print rx, ry, rz