'''
Created on 22 dec. 2018

@author: PASTOR Robert
'''

import numpy as np
import math
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import sys

class GUI():
    # 'quad_list' is a dictionary of format: quad_list = {'quad_1_name':{'position':quad_1_position,'orientation':quad_1_orientation,'arm_span':quad_1_arm_span}, ...}
    def __init__(self, quads, plannedTrajectory):
        self.quads = quads
        self.fig = plt.figure()
        self.ax = Axes3D.Axes3D(self.fig)
        
        ''' compute axes limits from planned trajectory '''
        self.ax.set_xlim3d([-10.0, 10.0])
        self.ax.set_xlabel('X meters')
        self.ax.set_ylim3d([-10.0, 10.0])
        self.ax.set_ylabel('Y meters')
        self.ax.set_zlim3d([0, 20.0])
        self.ax.set_zlabel('Z meters')
        
        if (isinstance(plannedTrajectory, list)):
            self.ax.set_xlim3d([min(map(lambda x: x[0], plannedTrajectory)) - 5., max(map(lambda x: x[0], plannedTrajectory)) + 5.])
            self.ax.set_ylim3d([min(map(lambda x: x[1], plannedTrajectory)) - 5., max(map(lambda x: x[1], plannedTrajectory)) + 5.])
            self.ax.set_zlim3d([min(map(lambda x: x[2], plannedTrajectory)) - 5., max(map(lambda x: x[2], plannedTrajectory)) + 5.])
        
        self.ax.set_title('Quadcopter Simulation')
        self.init_plot()
        self.fig.canvas.mpl_connect('key_press_event', self.keypress_routine)

    def rotation_matrix(self,angles):
        ct = math.cos(angles[0])
        cp = math.cos(angles[1])
        cg = math.cos(angles[2])
        st = math.sin(angles[0])
        sp = math.sin(angles[1])
        sg = math.sin(angles[2])
        R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
        R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R

    def init_plot(self):
        for key in self.quads:
            self.quads[key]['l1'], = self.ax.plot([],[],[], color='blue', linewidth=3, antialiased=False)
            self.quads[key]['l2'], = self.ax.plot([],[],[], color='red', linewidth=3, antialiased=False)
            self.quads[key]['hub'], = self.ax.plot([],[],[], marker='o',color='green', markersize=6,antialiased=False)

    def update(self):
        ''' loop through all the quads '''
        for key in self.quads:
            R = self.rotation_matrix(self.quads[key]['orientation'])
            ''' quad arm length '''
            L = self.quads[key]['L']
            ''' correct to arm length to one part of the world limits '''
            L = 2.0
            points = np.array([ [-L,0,0], [L,0,0], [0,-L,0], [0,L,0], [0,0,0], [0,0,0] ]).T
            points = np.dot(R,points)
            points[0,:] += self.quads[key]['position'][0]
            points[1,:] += self.quads[key]['position'][1]
            points[2,:] += self.quads[key]['position'][2]
            self.quads[key]['l1'].set_data(points[0,0:2],points[1,0:2])
            self.quads[key]['l1'].set_3d_properties(points[2,0:2])
            self.quads[key]['l2'].set_data(points[0,2:4],points[1,2:4])
            self.quads[key]['l2'].set_3d_properties(points[2,2:4])
            self.quads[key]['hub'].set_data(points[0,5],points[1,5])
            self.quads[key]['hub'].set_3d_properties(points[2,5])
        plt.pause(0.000000000000001)

    def keypress_routine(self,event):
        sys.stdout.flush()
        if event.key == 'x':
            y = list(self.ax.get_ylim3d())
            y[0] += 0.2
            y[1] += 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'w':
            y = list(self.ax.get_ylim3d())
            y[0] -= 0.2
            y[1] -= 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'd':
            x = list(self.ax.get_xlim3d())
            x[0] += 0.2
            x[1] += 0.2
            self.ax.set_xlim3d(x)
        elif event.key == 'a':
            x = list(self.ax.get_xlim3d())
            x[0] -= 0.2
            x[1] -= 0.2
            self.ax.set_xlim3d(x)


def main():
    plannedTrajectory = [(0.0 , -4.0 , 6.0 ), (5.0, 0.0 , 4.0),  (0.0 , 0.0, 2.0)]
    print ' Max X = {0}'.format(max(map(lambda x: x[0], plannedTrajectory)))
    print ' Min X = {0}'.format(min(map(lambda x: x[0], plannedTrajectory)))
    
    print ' Max Y = {0}'.format(max(map(lambda x: x[1], plannedTrajectory)))
    print ' Min Y = {0}'.format(min(map(lambda x: x[1], plannedTrajectory)))
    
    print ' Max Z = {0}'.format(max(map(lambda x: x[2], plannedTrajectory)))
    print ' Min Z = {0}'.format(min(map(lambda x: x[2], plannedTrajectory)))

if __name__ == "__main__":
    main()