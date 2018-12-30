'''
Created on 30 avr. 2018

@author: PASTOR Robert
'''

import numpy
import math
import random

from ForcesTorques import acceleration, angular_acceleration
from Controller import pid_controller

''' Simulation times, in seconds. '''
start_time = 0;
end_time = 10;
dt = 0.005;
times = [start_time,dt,end_time];

''' % Number of points in the simulation. '''
''' 'N = numel(times); '''

''' % Initial simulation state. '''
x = [0, 0, 10];
xdot = numpy.zeros(3, 1);
theta = numpy.zeros(3, 1);

'''% Simulate some disturbance in the angular velocity. '''
''' % The magnitude of the deviation is in radians / second. '''
deviation = 100;
'''thetadot = deg2rad(2 * deviation * rand(3, 1) - deviation); '''
thetadot = math.radians(2 * deviation * random.random(3, 1) - deviation);

'''% Step through the simulation, updating the state. '''
''' for t in times: '''
for t in numpy.arange(start_time, end_time + dt, dt):
    ''' % Take input from our controller. '''
    print t
    i = input(t);

    omega = thetadot2omega(thetadot, theta);

    ''' % Compute linear and angular accelerations. '''
    a = acceleration(i, theta, xdot, m, g, k, kd);
    omegadot = angular_acceleration(i, omega, I, L, b, k);

    omega = omega + dt * omegadot;
    thetadot = omega2thetadot(omega, theta); 
    theta = theta + dt * thetadot;
    xdot = xdot + dt * a;
    x = x + dt * xdot;
''' end '''
    
    
def cost(theta):
    '''% Create a controller using the given gains. '''
    control = controller('pid', theta(1), theta(2), theta(3));

    ''' % Perform a simulation. '''
    data = simulate(control);

    ''' % Compute the integral, $\frac{1}{t_f - t_0} \int_{t_0}^{t_f} e(t)^2 dt$ '''
    t0 = 0;
    tf = 1;
    J = 1/(tf - t0) * sum(data.theta(data.t >= t0 & data.t <= tf) .^ 2) * data.dt;
    return J
''' end '''