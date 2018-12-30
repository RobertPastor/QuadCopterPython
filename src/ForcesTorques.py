'''
Created on 30 avr. 2018

@author: PASTOR Robert
'''

import numpy

''' % Compute thrust given current inputs and thrust coefficient. '''
def thrust(inputs, k):
    ''' % Inputs are values for ${\omega_i}^2$ '''
    T = [0, 0, k * sum(inputs)];
    return T
''' end '''


''' % Compute torques, given current inputs, length, drag coefficient, and thrust coefficient. '''
def torques(inputs, L, b, k):
    ''' % Inputs are values for ${\omega_i}^2$ '''
    tau = [
        L * k * (inputs(1) - inputs(3)),
        L * k * (inputs(2) - inputs(4)),
        b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
    ];
    return tau
''' end '''


def acceleration(inputs, angles, xdot, m, g, k, kd):
    gravity = [0, 0, -g];
    R = rotation(angles);
    T = R * thrust(inputs, k);
    Fd = -kd * xdot;
    a = gravity + 1 / m * T + Fd;
    return a
''' end '''


def angular_acceleration(inputs, omega, I, L, b, k):
    tau = torques(inputs, L, b, k);
    ''' inverse matrix I '''
    InverseI = numpy.linalg.inv(I)
    omegaddot = InverseI * (tau - numpy.cross(omega, I * omega));
    return omegaddot
''' end '''