import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates, LinkStates
from matplotlib import pyplot as plt
from controllers import PID_controller, state
waypoints = []


def main():
    ###########################
    #
    # initialise variables 
    # identify target waypoint
    # solve the thruster and heading problem(desired heading, the ship shouldn't drift, yaw des = yaw of veh)
    # 
    ########################### 

    #collect localisation data


    PID_steer = PID_controller()
    PID_speed = PID_controller()
    idx = 0
    while idx is not range(len(waypoints)):    
        a = rospy.wait_for_message("/gazebo/model_states",ModelStates,timeout=1)
        WAMV_pos = a.poses[len(a.poses)-1].x a.poses[len(a.poses)-1].y a.poses[len(a.poses)-1].yaw     
        if np.hypot(veh_s.x - waypoints[idx][0], veh_s.y - waypoints[idx][1]) < 3:
            idx += 1
        