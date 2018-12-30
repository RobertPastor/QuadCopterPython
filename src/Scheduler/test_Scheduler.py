# -*- coding: utf-8 -*-
'''
Created on 21 dec. 2018

@author: PASTOR Robert
'''
from src.Scheduler.scheduler import Scheduler

import time


def main():
    
    scheduler = Scheduler()
    start = time.time()
    
    task1 = scheduler.add_task(computePositionSpeedAcceleration, 0.1 , (start,))
    kEvent_Render = scheduler.add_event(rendering)
    task2 = scheduler.add_task(computePlannedTrajectory, 1.0 , (start,))
    scheduler.start()
    
    try:
        while True:
            scheduler.postEvent(kEvent_Render)
            time.sleep(1)
    except KeyboardInterrupt:
        print ("attempting to close threads.")
        scheduler.stop()
        print ("terminated.")


def computePositionSpeedAcceleration(start):
    now = time.time()
    print "mesurer la position, la vitesse et l'accélération - secondes= {0}".format(now - start)

def computePlannedTrajectory(start):
    now = time.time()
    print "calculer la trajectoire planifiée - secondes= {0}".format(now - start)

def rendering():
    now = time.time()
    print "visualiser la trajectoire"

if __name__ == '__main__':
    main()