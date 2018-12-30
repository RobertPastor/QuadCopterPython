'''
Created on 9 dec. 2018

@author: PASTOR Robert
'''

import numpy as np
import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import Axes3D


nb = 30
nb = 10
nb = 30

u = np.linspace(0, np.pi, nb)
v = np.linspace(0, 2 * np.pi, nb)

degrees = 90
x = degrees * np.outer(np.sin(u), np.sin(v))
y = degrees * np.outer(np.sin(u), np.cos(v))
z = degrees * np.outer(np.cos(u), np.ones_like(v))

fig = plt.figure()
ax = plt.axes(projection='3d')

ax.plot_wireframe(x, y, z, rcount=6, ccount=6)

plt.show()