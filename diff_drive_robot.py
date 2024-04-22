# Author: Mark Moll, modified by Bijo Sebastian

from math import sin, cos
import numpy as np
from functools import partial

from ompl import base as ob
from ompl import control as oc

  
def isStateValid(spaceInformation, state):
    #Function to check validity of state
    #IMPLEMENT THIS FUNCTION: it should return True or False only
    return False
  
def propagate(state_in, control, duration, state_out):
    #Use the sample controls to propogate to new state
    #IMPLEMENT THIS FUNCTION TO PROPOGATE FROM state_in GIVEN THE CONTROL INPUT control FOR THE DURATION duration AND ASSIGN THE RESULTANT STATE IN state_out
