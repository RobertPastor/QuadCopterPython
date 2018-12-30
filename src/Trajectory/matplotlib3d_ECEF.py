# -*- coding: utf-8 -*-
'''
Created on 9 dec. 2018

@author: PASTOR Robert
'''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits import mplot3d
from matplotlib.text import Annotation
import matplotlib.ticker as ticker

plt.rcParams.update({'font.size': 9})

class Annotation3D(Annotation):
    '''Annotate the point xyz with text s'''

    def __init__(self, s, xyz, *args, **kwargs):
        Annotation.__init__(self,s, xy=(0,0), *args, **kwargs)
        self._verts3d = xyz        

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = mplot3d.proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.xy=(xs,ys)
        Annotation.draw(self, renderer)
        

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = mplot3d.proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

nb = 30
theta = np.linspace(0, 2 * np.pi, nb)
print theta
phi = np.linspace(0, np.pi, nb)

THETA, PHI = np.meshgrid(theta, phi)

R = 90
X = R * np.sin(PHI) * np.cos(THETA)
Y = R * np.sin(PHI) * np.sin(THETA)
Z = R * np.cos(PHI)

fig = plt.figure()

ax = fig.add_subplot(1,1,1, projection='3d')
#ax.plot_surface(X, Y, Z, rstride=1, cstride=1, linewidth=0.3, color='white', antialiased=False, alpha=0.51)
ax.plot_surface(X, Y, Z, linewidth=0.3, color='white', antialiased=False, alpha=0.5)

nb = 60

u = np.linspace(0, np.pi, nb)
v = np.linspace(0, 2 * np.pi, nb)

degrees = 90
x = degrees * np.outer(np.sin(u), np.sin(v))
y = degrees * np.outer(np.sin(u), np.cos(v))
z = degrees * np.outer(np.cos(u), np.ones_like(v))

#//fig = plt.figure()
#ay = plt.axes(projection='3d')

#ay = fig.add_subplot(1,1,1, projection='3d')
ax.plot_wireframe(x, y, z, rcount=6, ccount=2, linewidth=1.5 , color='blue' )

maxDegrees = 100
refx0 = [0, 0, 0]
refx1 = [+ maxDegrees, 0 ,0]
#ax.plot(refx0, refx1, linewidth=1.5, color='black')
arrow1 = Arrow3D([refx0[0],refx1[0]], [refx0[1], refx1[1]], [refx0[2], refx1[2]], mutation_scale=20, linewidth=1.5, arrowstyle="-|>", color="red")
ax.add_artist(arrow1)

tagY = Annotation3D("Y", refx1 , fontsize=10, xytext=(-3,3), textcoords='offset points', ha='right',va='bottom')
ax.add_artist(tagY)

refy0 = [0 , 0 , 0]
refy1 = [0 , - maxDegrees , 0]
arrow2 = Arrow3D([refy0[0],refy1[0]], [refy0[1], refy1[1]], [refy0[2], refy1[2]], mutation_scale=20, linewidth=1.5, linestyle='dashed', arrowstyle="-|>", color="black")
ax.add_artist(arrow2)

tagX = Annotation3D("X", refy1, fontsize=10, xytext=(-3,3), textcoords='offset points', ha='left',va='top')
ax.add_artist(tagX)

refz0 = [0 , 0 , 0]
refz1 = [0 , 0 , + maxDegrees]
arrow3 = Arrow3D([refz0[0],refz1[0]], [refz0[1], refz1[1]], [refz0[2], refz1[2]], mutation_scale=20, linewidth=1.5, linestyle='dashdot', arrowstyle="-|>", color="green")
ax.add_artist(arrow3)

tagZ = Annotation3D("Z", refz1, fontsize=10, xytext=(-3,3), textcoords='offset points', ha='right',va='bottom')
ax.add_artist(tagZ)

formatter = ticker.FormatStrFormatter('%2.0f' + unicode("°" , encoding='utf8') )
ax.xaxis.set_major_formatter(formatter)
ax.yaxis.set_major_formatter(formatter)
ax.zaxis.set_major_formatter(formatter)

xtext = ax.set_zlabel('Latitude') # returns a Text instance

ax.set_xlim(-90, +90)
ax.set_ylim(-90, +90)
ax.set_zlim(-90, +90)


refMeridien = [0 , - maxDegrees , + maxDegrees/3]
tagMeridien = Annotation3D(unicode("Méridien de Greenwich", encoding='utf8'), refMeridien, xycoords='data',
            xytext=(-15, 25), textcoords='offset points',
            arrowprops=dict(facecolor='black', shrink=0.05),
            horizontalalignment='right', verticalalignment='bottom')
#                           fontsize=10, xytext=(-3,3), textcoords='offset points', ha='right',va='bottom')
ax.add_artist(tagMeridien)

refEquateur = [ + maxDegrees/2 , - maxDegrees , 0]
tagEquateur = Annotation3D(unicode("Equateur", encoding='utf8'), refEquateur, xycoords='data',
            xytext=(-15, 25), textcoords='offset points',
            arrowprops=dict(facecolor='black', shrink=0.05),
            horizontalalignment='left', verticalalignment='bottom')
ax.add_artist(tagEquateur)

#plt.axis('equal')
ax.view_init(elev=8, azim=-77) #Works!

plt.show()