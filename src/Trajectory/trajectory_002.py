# -*- coding: utf-8 -*-
'''
Created on 16 dec. 2018

@author: PASTOR Robert
'''
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion
from src.Trajectory.bezier_curve import bezier_curve

class Position():
    
    def __init__(self, _x, _y, _z):
        self.x = _x
        self.y = _y
        self.z = _z
        
    def getX(self):
        return self.x
    
    def getY(self):
        return self.y
    
    def getZ(self):
        return self.z


class Trajectory():
    
    def __init__(self):
        self.initialPosition = Position(0.0 , 0.0, 0.0)
        self.lastPosition = self.initialPosition
        self.wayPoints = []
        self.wayPoints.append(self.initialPosition)
        self.headingDegrees = 0.0
        pass

    def buildTrajectory(self):
        print '---------build trajectory-------------'
        self.climb(targetAltitudeMeters=5.0 , numberOfPoints=10)
        self.climb_n_progress(directionDegrees=45.0, targetAltitudeMeters=10.0, distanceMeters=50.0, numberOfPoints=10)
        #self.turn(targetHeadingDegrees=90.0, radiusMeters=50.0)
        self.climb_n_progress(directionDegrees=90.0, targetAltitudeMeters=20.0, distanceMeters=200.0, numberOfPoints=10)
        #self.turn(targetHeadingDegrees=180.0, radiusMeters=50.0)
        self.climb_n_progress(directionDegrees=180.0, targetAltitudeMeters=20.0, distanceMeters=200.0, numberOfPoints=10)
        #self.turn(targetHeadingDegrees=0.0, radiusMeters=50.0)
        self.climb_n_progress(directionDegrees=-90.0, targetAltitudeMeters=20.0, distanceMeters=300.0, numberOfPoints=10)
        self.climb_n_progress(directionDegrees=0.0, targetAltitudeMeters=10.0, distanceMeters=100.0, numberOfPoints=10)
        self.climb_n_progress(directionDegrees=0.0, targetAltitudeMeters=0.0, distanceMeters=25.0, numberOfPoints=10)
        
    
    def climb(self, targetAltitudeMeters, numberOfPoints):
        initialZ = self.lastPosition.getZ()
        zList = np.linspace(initialZ, targetAltitudeMeters, numberOfPoints)
        print zList
        for z in zList:
            pos = Position(self.lastPosition.getX(), self.lastPosition.getY(), z)
            self.wayPoints.append(pos)
            self.lastPosition = pos
    

    def climb_n_progress(self, directionDegrees, targetAltitudeMeters, distanceMeters, numberOfPoints):
        initialZ = self.lastPosition.getZ()
        descentClimbAngleRadians = math.atan( (targetAltitudeMeters-initialZ) / distanceMeters)
        
        print 'climb angle= {0} degrees'.format(math.degrees(descentClimbAngleRadians))
        distanceList = np.linspace(0.0, distanceMeters, numberOfPoints)
        for dist in distanceList:
            print dist
            x = dist * math.cos(math.radians(directionDegrees))
            y= dist * math.sin(math.radians(directionDegrees))
            
            z = initialZ + math.tan(descentClimbAngleRadians) * dist
            pos = Position(self.lastPosition.getX()+x, self.lastPosition.getY()+y, z)
            self.wayPoints.append(pos)
            self.lastPosition = pos
            
        self.headingDegrees = directionDegrees;
    
    
    def turn(self, targetHeadingDegrees, radiusMeters):
        print '-----initial heading= {0} degrees'.format(self.headingDegrees)
        initialHeadingDegrees = self.headingDegrees;
        '''q0 = Quaternion(axis=[1, 0, 1], angle=math.radians(0.0))
        q1 = Quaternion(axis=[1, 0, 1], angle=math.radians(180.))
        for q in Quaternion.intermediates(q0, q1, 10, include_endpoints=True):
            v = q.rotate([0, 0, 1])
            pos = Position(v[0] * radiusMeters, v[1] * radiusMeters, v[2] * radiusMeters)
            self.wayPoints.append(pos)
            print(v)
        '''
        nbPoints = int(abs(targetHeadingDegrees-initialHeadingDegrees) )
        direction = +1
        if (targetHeadingDegrees < initialHeadingDegrees):
            direction = -1
        for t in range(0, nbPoints + 1):
            print t
            pos = Position ( self.lastPosition.getX() + math.cos(math.radians(initialHeadingDegrees + (t*direction))) * radiusMeters , self.lastPosition.getY() + math.sin (math.radians(initialHeadingDegrees + (t*direction))) * radiusMeters , self.lastPosition.getZ())
            self.wayPoints.append(pos)
            self.lastPosition = pos
        
        self.headingDegrees = targetHeadingDegrees
    
    def plotBezier(self):
        fig = plt.figure()
        ax = Axes3D(fig)
        #ax = fig.add_subplot(1,1,1, projection='3d')
        
        points = []
        for wayPoint in self.wayPoints:
            points.append([wayPoint.getX(), wayPoint.getY(), wayPoint.getZ()])
            
        xvals, yvals , zvals = bezier_curve(points, nTimes=1000)
        ax.scatter(xvals, yvals, zvals)
        
        ax.set(xlabel=unicode('mètres',encoding='utf8'), ylabel=unicode('mètres',encoding='utf8'), zlabel=unicode('mètres',encoding='utf8'), title=unicode('Example of de trajectoire planifiée', encoding='utf8'))
        
        #for nr in range(len(points)):
        #    plt.text(points[nr][0], points[nr][1], nr)
        plt.show()

    
    def plotTrajectory(self):
        print '---------plot trajectory------------'

        fig = plt.figure()
        ax = fig.add_subplot(1,1,1, projection='3d')

        counter = 1
        for waypoint in self.wayPoints:
            ax.scatter(waypoint.getX(), waypoint.getY(), waypoint.getZ(), color='blue' )

            #ax.text(waypoint.getX(), waypoint.getY(), waypoint.getZ()+0.3, str(counter))
            counter = counter + 1
            
        plt.show()


def main():
    print '-------main-------'
    trajectory = Trajectory()
    trajectory.buildTrajectory()
    trajectory.plotBezier()
    #trajectory.plotTrajectory()
    pass


if __name__ == "__main__":
    main()
    
    