'''
Created on 10 dec 2018

@author: PASTOR Robert
'''
from pyquaternion import Quaternion
import math

q1 = Quaternion(axis=[0.9501, 0.1675, -0.2630], degrees=67.6836)
print q1.rotation_matrix

print '========================'
q2 = Quaternion(axis=[0 , 0 , 1], degrees=20.)
q3 = Quaternion(axis=[1 , 0 , 1], degrees=-65.)

q4 = q3 * q2
print q4.rotation_matrix
q5 = q2 * q3
print q5.rotation_matrix

print '========================'

q6 = Quaternion(scalar=0.9239, vector=(-0.3827 , 0 , 0))
q7 = Quaternion(scalar=0.9239, vector=(0 , 0.3827 , 0))
print q6*q7
print '========================'
q8 = Quaternion(scalar=0.9239, vector=(0 , 0 , -0.3827 ))
print '========================'

q9 = (q6 * q7) * q8
print q9.rotation_matrix

print '================='
q0 = Quaternion(axis=[1 , 1, 1], angle=math.radians(45.0))
q1 = Quaternion(axis=[1 , 1, 1], angle=math.radians(90.0))
for q in Quaternion.intermediates(q0, q1, 10, include_endpoints=True):
    v = q.rotate([0, 0, 1])
    print(v)