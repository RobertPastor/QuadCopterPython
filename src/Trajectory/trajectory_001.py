'''
Created on 8 december. 2018

@author: PASTOR Robert
'''

import numpy as np
from numpy.linalg import inv
from collections import namedtuple
from numpy import linalg as LA
import matplotlib.pyplot as plt

DesiredState = namedtuple('DesiredState', 'pos vel acc yaw yawdot')
yaw = 0.0
current_heading = np.zeros(2)

def get_helix_waypoints(t, n):
    """ The function generate n helix waypoints from the given time t
        output waypoints shape is [n, 3]
    """
    waypoints_t = np.linspace(t, t + 2*np.pi, n)
    x = 0.5*np.cos(waypoints_t)
    y = 0.5*np.sin(waypoints_t)
    z = waypoints_t

    return np.stack((x, y, z), axis=-1)


def main():
    waypoints = get_helix_waypoints(0, 50)
    x = []
    y = []
    z = []
    for waypoint in waypoints:
        print waypoint
        print waypoint[0] , waypoint[1]
        x.append(waypoint[0])
        y.append(waypoint[1])
        z.append(waypoint[2])
    print waypoints
    
    fig, ax = plt.subplots()
    ax.plot(x, y)

    ax.set(xlabel='x', ylabel='y',
    title='About as simple as it gets, folks')
    ax.grid()

    plt.show()
    
if __name__ == "__main__":
    print np.version.version
    main()