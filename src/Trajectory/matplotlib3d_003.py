'''
Created on 9 dec. 2018

@author: PASTOR Robert
'''

'''
========================
3D surface (solid color)
========================

Demonstrates a very basic plot of a 3D surface using a solid color.
'''

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np


fig = plt.figure()

ax = fig.add_subplot(1,1,1, projection='3d')

# Make data
nb = 30
u = np.linspace(0, 2 * np.pi, nb)
print u
v = np.linspace(0, np.pi, nb)
print v

degrees = 90
x = degrees * np.outer(np.cos(u), np.sin(v))
y = degrees * np.outer(np.sin(u), np.sin(v))
z = degrees * np.outer(np.ones(np.size(u)), np.cos(v))

# Plot the surface
ax.plot_surface(x, y, z)

ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

plt.show()
