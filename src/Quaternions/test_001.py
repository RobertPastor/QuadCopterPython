'''
Created on 8 december. 2018

@author: PASTOR Robert
'''

from pyquaternion import Quaternion

''' (2 - i  +  j  + 3 k ) * ( -1 + i  + 4j  - 2 k ) '''

q1 = Quaternion(2, -1, 1, 3)
q2 = Quaternion(-1, 1, 4 , -2)

print q1.__mul__(q2)
print q1*q2
print '=========='
print q2.__mul__(q1)

print '=========='
qi = Quaternion(0, 1, 0, 0)
qj = Quaternion(0, 0, 1, 0)
print qi*qj

qk = Quaternion(0, 0, 0, 1)
print qi*qj == qk
