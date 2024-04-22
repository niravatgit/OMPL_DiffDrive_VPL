# Author: Mark Moll, modified by Bijo Sebastian

from math import sin, cos
import numpy as np
from functools import partial
import diff_drive_robot as ddr

from ompl import base as ob
from ompl import control as oc

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

#Defining axes and figure for plotting 
fig, ax1 = plt.subplots()
ax1.set(xlabel='X(m)', ylabel='Y(m)') #setting X and Y labels 
  
def plan():
    # construct the state space we are planning in
    # SE2 --> Special Euclidean x, y in R and yaw in orientation
    space = ob.SE2StateSpace()
  
    # set the bounds for the R^2 part of SE(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(10)
    space.setBounds(bounds)
  
    # create a control space
    # Control space has real vectors for V and omega control inputs
    cspace = oc.RealVectorControlSpace(space, 2)
  
    # set the bounds for the control space
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(0,0)
    cbounds.setHigh(0, 1.0)
    cbounds.setLow(1, -0.1)
    cbounds.setHigh(1, 0.1)
    cspace.setBounds(cbounds)
  
    # define a simple setup class
    ss = oc.SimpleSetup(cspace)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(partial(ddr.isStateValid, ss.getSpaceInformation())))
    ss.setStatePropagator(oc.StatePropagatorFn(ddr.propagate))
  
    #Create and set start state
    start = ob.State(space)
    start().setX(0.0)
    start().setY(0.0)
    start().setYaw(np.pi/2.0)
  
    #Create and set goal state
    goal = ob.State(space)
    goal().setX(0.0)
    goal().setY(10.0)
    goal().setYaw(np.pi)
  
    # set the start and goal states along with goal tolerance
    ss.setStartAndGoalStates(start, goal, 0.005)

    #Plot start and goal point as green and blue 
    ax1.scatter(start[0], start[1], alpha=1.0, s=75, color='green') 
    ax1.scatter(goal[0], goal[1], alpha=1.0, s=75, color='blue') 
    
    #Plot obstacles
    obstacle1 = Rectangle(xy = (0, 2), width = 7.0, height = 3.0, angle = 0.0, edgecolor='k', fc='k', lw=2)
    ax1.add_patch(obstacle1)
    obstacle2 = Rectangle(xy = (4, 7), width = 6.0, height = 3.0, angle = 0.0, edgecolor='k', fc='k', lw=2)
    ax1.add_patch(obstacle2)

    #Get space information needed for setting the options below
    space_information = ss.getSpaceInformation()
    # set planner, KPIECE1 is default
    planner = oc.RRT(space_information)
    ss.setPlanner(planner)
    # set propagation step size
    space_information.setPropagationStepSize(0.5)
    # attempt to solve the problem in not more than 20 seconds 
    solved = ss.solve(20.0)
  
    if solved:
        # Get the solution path 
        path = ss.getSolutionPath()

        #Get solution 
        prev_waypoint_state = [0,0]
        for way_point_id in range(path.getStateCount()):
            waypoint_state = path.getState(way_point_id)
            print("Waypoint number:",way_point_id,"[", waypoint_state.getX(), waypoint_state.getY(), "]")
            ax1.plot([waypoint_state.getX(), prev_waypoint_state[0]], [waypoint_state.getY(), prev_waypoint_state[1]],'r--') 
            ax1.scatter(waypoint_state.getX(), waypoint_state.getY(), alpha=1.0, s=25, color='red') 
            prev_waypoint_state = [waypoint_state.getX(), waypoint_state.getY()]
    plt.show()
        
if __name__ == "__main__":
    plan()
