'''
Created on 22 dec. 2018

@author: PASTOR Robert
'''

#import quadcopter,gui,controller
from src.Quadcopter.Quadcopter import Quadcopter
from src.Quadcopter.Controller import Controller_PID_Point2Point, Controller_PID_Velocity
from src.Quadcopter.Gui import GUI
from src.Quadcopter.Trajectory import Trajectory , Position
import signal
import sys
import argparse

# Constants
#TIME_SCALING = 1.0 # Any positive number(Smaller is faster). 1.0->Real Time, 0.0->Run as fast as possible
TIME_SCALING = 0.1
QUAD_DYNAMICS_UPDATE = 0.01 # seconds
CONTROLLER_DYNAMICS_UPDATE = 0.001 # seconds
run = True

def Single_Point2Point():
    # Set goals to go to
    GOALS = [(1,1,2),(1,-1,4),(-1,-1,2),(-1,1,4)]
    YAWS = [0,3.14,-1.54,1.54]
    # Define the quadcopters
    QUADCOPTER={'q1':{'position':[1,0,4],'orientation':[0,0,0],'L':0.3,'r':0.1,'prop_size':[10,4.5],'weight':1.2}}
    # Controller parameters
    CONTROLLER_PARAMETERS = {'Motor_limits':[4000,7000],
                        'Tilt_limits':[-10,10],
                        'Yaw_Control_Limits':[-900,900],
                        'Z_XY_offset':500,
                        'Linear_PID':{'P':[300, 300, 7000],'I':[0.04, 0.04, 4.5],'D':[450, 450, 5000]},
                        'Linear_To_Angular_Scaler':[1,1,0],
                        'Yaw_Rate_Scaler':0.18,
                        'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                        }

    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, signal_handler)
    # Make objects for quadcopter, gui and controller
    quad = Quadcopter(QUADCOPTER)
    gui_object = GUI(quads=QUADCOPTER)
    ctrl = Controller_PID_Point2Point(quad.get_state,quad.get_time,quad.set_motor_speeds,params=CONTROLLER_PARAMETERS,quad_identifier='q1')
    # Start the threads
    quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    ctrl.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    # Update the GUI while switching between destination positions
    while(run==True):
        for goal,y in zip(GOALS,YAWS):
            ctrl.update_target(goal)
            ctrl.update_yaw_target(y)
            for i in range(300):
                gui_object.quads['q1']['position'] = quad.get_position('q1')
                gui_object.quads['q1']['orientation'] = quad.get_orientation('q1')
                gui_object.update()
    quad.stop_thread()
    ctrl.stop_thread()

def Multi_Point2Point():
    # Set goals to go to
    GOALS_1 = [(-1,-1,4),(1,1,2)]
    GOALS_2 = [(1,-1,2),(-1,1,4)]
    # Define the quadcopters
    QUADCOPTERS={'q1':{'position':[1,0,4],'orientation':[0,0,0],'L':0.3,'r':0.1,'prop_size':[10,4.5],'weight':1.2},
        'q2':{'position':[-1,0,4],'orientation':[0,0,0],'L':0.15,'r':0.05,'prop_size':[6,4.5],'weight':0.7}}
    # Controller parameters
    CONTROLLER_1_PARAMETERS = {'Motor_limits':[4000,9000],
                        'Tilt_limits':[-10,10],
                        'Yaw_Control_Limits':[-900,900],
                        'Z_XY_offset':500,
                        'Linear_PID':{'P':[300,300,7000],'I':[0.04,0.04,4.5],'D':[450,450,5000]},
                        'Linear_To_Angular_Scaler':[1,1,0],
                        'Yaw_Rate_Scaler':0.18,
                        'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                        }
    CONTROLLER_2_PARAMETERS = {'Motor_limits':[4000,9000],
                        'Tilt_limits':[-10,10],
                        'Yaw_Control_Limits':[-900,900],
                        'Z_XY_offset':500,
                        'Linear_PID':{'P':[300,300,7000],'I':[0.04,0.04,4.5],'D':[450,450,5000]},
                        'Linear_To_Angular_Scaler':[1,1,0],
                        'Yaw_Rate_Scaler':0.18,
                        'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                        }

    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, signal_handler)
    # Make objects for quadcopter, gui and controllers
    gui_object = GUI(quads=QUADCOPTERS)
    quad = Quadcopter(quads=QUADCOPTERS)
    ctrl1 = Controller_PID_Point2Point(quad.get_state,quad.get_time,quad.set_motor_speeds,params=CONTROLLER_1_PARAMETERS,quad_identifier='q1')
    ctrl2 = Controller_PID_Point2Point(quad.get_state,quad.get_time,quad.set_motor_speeds,params=CONTROLLER_2_PARAMETERS,quad_identifier='q2')
    # Start the threads
    quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    ctrl1.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    ctrl2.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    # Update the GUI while switching between destination poitions
    while(run==True):
        for goal1,goal2 in zip(GOALS_1,GOALS_2):
            ctrl1.update_target(goal1)
            ctrl2.update_target(goal2)
            for i in range(150):
                for key in QUADCOPTERS:
                    gui_object.quads[key]['position'] = quad.get_position(key)
                    gui_object.quads[key]['orientation'] = quad.get_orientation(key)
                gui_object.update()
    quad.stop_thread()
    ctrl1.stop_thread()
    ctrl2.stop_thread()


def Single_Velocity(trajectoryList):
    ''' one quad single speed '''
    # Set goals to go to
    GOALS = [(0.5,0,2),(0,0.5,2),(-0.5,0,2),(0,-0.5,2)]
    GOALS = trajectoryList
    # Define the quadcopters
    QUADCOPTER={'q1':{'position':[0,0,0],'orientation':[0,0,0],'L':0.3,'r':0.1,'prop_size':[10,4.5],'weight':1.2}}
    # Controller parameters
    CONTROLLER_PARAMETERS = {'Motor_limits':[4000,9000],
                        'Tilt_limits':[-10,10],
                        'Yaw_Control_Limits':[-900,900],
                        'Z_XY_offset':500,
                        'Linear_PID':{'P':[2000,2000,7000],'I':[0.25,0.25,4.5],'D':[50,50,5000]},
                        'Linear_To_Angular_Scaler':[1,1,0],
                        'Yaw_Rate_Scaler':0.18,
                        'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                        }

    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, signal_handler)
    # Make objects for quadcopter, gui and controller
    quad = Quadcopter(QUADCOPTER)
    gui_object = GUI(quads=QUADCOPTER)
    ctrl = Controller_PID_Velocity(quad.get_state,quad.get_time,quad.set_motor_speeds,params=CONTROLLER_PARAMETERS,quad_identifier='q1')
    # Start the threads
    quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    ctrl.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    # Update the GUI while switching between destination positions
    while(run==True):
        for goal in GOALS:
            print 'goal= {0}'.format(goal)
            targetPos = Position(goal[0], goal[1], goal[2])
            ctrl.update_target(goal)
            currentPos = Position(quad.get_position('q1')[0],quad.get_position('q1')[1],quad.get_position('q1')[2])
            while(currentPos.isNear(targetPos, 0.5) == False):
                gui_object.quads['q1']['position'] = quad.get_position('q1')
                gui_object.quads['q1']['orientation'] = quad.get_orientation('q1')
                gui_object.update()
    quad.stop_thread()
    ctrl.stop_thread()
    
    
def simulationPointToPoint(plannedTrajectory):
    ''' one quad single speed '''
    # Set goals to go to
    
    GOALS = plannedTrajectory
    #GOALS = trajectoryList
    # Define the quadcopters
    QUADCOPTER={'q1':{'position':[0,0,0],'orientation':[0,0,0],'L':0.3,'r':0.1,'prop_size':[10, 4.5],'weight':1.2}}
    # Controller parameters 
    ''' tilt limits in degrees - converted in radians in the controller '''
    CONTROLLER_PARAMETERS = {'Motor_limits':[4000,9000],
                        'Tilt_limits':[-5.0,5.0],
                        'Yaw_Control_Limits':[-900,900],
                        'Z_XY_offset':500,
                        #'Linear_PID':{'P':[2000,2000,1500],'I':[0.25,0.25,4.5],'D':[50,50,5000]},
                        #'Linear_To_Angular_Scaler':[1,1,0],
                        #'Yaw_Rate_Scaler':0.18,
                        #'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                        'Linear_PID':{'P':[15000, 15000, 1500],'I':[100.25, 100.25, 4.5],'D':[3000, 3000, 1500]},
                        'Linear_To_Angular_Scaler':[1,1,0],
                        'Yaw_Rate_Scaler':0.18,
                        'Angular_PID':{'P':[22000,22000,1500],'I':[100.25,100.25,100.25],'D':[12000,12000,10]},
                        }

    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, signal_handler)
    # Make objects for quadcopter, gui and controller
    quad = Quadcopter(QUADCOPTER)
    gui_object = GUI(quads=QUADCOPTER, plannedTrajectory=plannedTrajectory)
    ctrl = Controller_PID_Velocity(quad.get_state,quad.get_time,quad.set_motor_speeds,params=CONTROLLER_PARAMETERS,quad_identifier='q1')
    # Start the threads
    quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    ctrl.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    # Update the GUI while switching between destination positions
    goalAchieved = False
    while(run==True):
        for goal in GOALS:
            
            goalAchieved = False
            nbIterations = 0
            nbMaxIterations = 10
            print 'goal= {0}'.format(goal)
            targetPos = Position(goal[0], goal[1], goal[2])
            ctrl.update_target(goal)
            
            while (goalAchieved == False):
            #while(currentPos.isNear(targetPos, radiusMeter = 0.5) == False):
                gui_object.quads['q1']['position'] = quad.get_position('q1')
                gui_object.quads['q1']['orientation'] = quad.get_orientation('q1')
                gui_object.update()
                quad.get_linear_rate('q1')
                
                currentPos = Position(quad.get_position('q1')[0],quad.get_position('q1')[1],quad.get_position('q1')[2])
                if currentPos.isNear(targetPos, radiusMeter = 0.25):
                    
                    nbIterations += 1
                    print 'goal is achieved - iterations = {0}'.format(nbIterations)
                    if (nbIterations > nbMaxIterations):
                        goalAchieved = True
                    
                
    quad.stop_thread()
    ctrl.stop_thread()
    

def parse_args():
    parser = argparse.ArgumentParser(description="Quadcopter Simulator")
    parser.add_argument("--sim", help='single_p2p, multi_p2p or single_velocity', default='single_p2p')
    parser.add_argument("--time_scale", type=float, default=-1.0, help='Time scaling factor. 0.0:fastest,1.0:realtime,>1:slow, ex: --time_scale 0.1')
    parser.add_argument("--quad_update_time", type=float, default=0.0, help='delta time for quadcopter dynamics update(seconds), ex: --quad_update_time 0.002')
    parser.add_argument("--controller_update_time", type=float, default=0.0, help='delta time for controller update(seconds), ex: --controller_update_time 0.005')
    return parser.parse_args()

def signal_handler(signal, frame):
    global run
    run = False
    print('Stopping')
    sys.exit(0)

if __name__ == "__main__":
    args = parse_args()
    if args.time_scale>=0: TIME_SCALING = args.time_scale
    if args.quad_update_time>0: QUAD_DYNAMICS_UPDATE = args.quad_update_time
    if args.controller_update_time>0: CONTROLLER_DYNAMICS_UPDATE = args.controller_update_time
    #if args.sim == 'single_p2p':
    #Single_Point2Point()
    #elif args.sim == 'multi_p2p':
    #    Multi_Point2Point()
    #elif args.sim == 'single_velocity':
    trajectory = Trajectory()
    trajectory.buildTrajectory()
    trajectory.computeBezier()
    plannedTrajectory = trajectory.getTrajectory()
    plannedTrajectory = [(0.0 , 0.0 , 6.0 ), (5.0, 3.0 , 4.0),  (-5.0 , 2.0, 2.0) , (0.0, 0.0, 4.0)]
    plannedTrajectory = [(0.0 , 0.0 , 6.0 ), (5.0, 0.0 , 5.0),  (5.0 , 5.0, 7.0) , (-5.0, 5.0, 4.0) , (-5.0, 0.0, 2.0) ]
    plannedTrajectory = [(0.0 , 0.0 , 6.0 ), (50.0, 0.0 , 5.0),  (50.0 , 50.0, 7.0) , (-50.0, 50.0, 4.0) , (-50.0, 0.0, 2.0) ]
    simulationPointToPoint(plannedTrajectory) 
