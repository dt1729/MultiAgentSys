from re import L
from statistics import mean
import time
from turtle import clear
from markupsafe import string
import numpy as np
import math
import rospy
import tf_conversions
from std_msgs.msg import Float32
from gazebo_msgs.msg import ModelStates, LinkStates
from matplotlib import pyplot as plt
from controllers import PID_controller, state
from cooperation_strategies import consensus, leader_model
#Circular waypoints
waypoint = [-500.0,290]
leader_waypoints = [[waypoint[0] + 10*math.cos(i), waypoint[1] + 10*math.sin(i)] for i in np.arange(0,2*math.pi,0.5)]

#straight Line waypoints

# waypoint = [[-500.0,290],[-460.0,320],[-540,300]]
# waypoints = []
# for j in range(1,len(waypoint)):
#     for i in np.arange(0,1,0.1):
#         waypoints.append([(1-i)*waypoint[j-1][0] + (i)*waypoint[j][0], (1-i)*waypoint[j-1][1] + (i)*waypoint[j][1]])

def retrieve_states():
    a = rospy.wait_for_message("/gazebo/link_states",LinkStates)
    wamv_pos = {}
    for i in range(len(a.name)):
        if a.name[i] == "wamv" + "::wamv" + "/imu_wamv_link":
            wamv_pos["wamv"] = [a.pose[i].position.x,a.pose[i].position.y,get_yaw_euler(a.pose[i])]
        for j in range(1,10):
            if a.name[i] == "wamv" + str(j) + "::wamv" + str(j)+"/imu_wamv_link":
                wamv_pos["wamv" + str(j)] = [a.pose[i].position.x,a.pose[i].position.y,get_yaw_euler(a.pose[i])]
    return wamv_pos

def get_yaw_euler(vehicle_pose):
    return tf_conversions.transformations.euler_from_quaternion([vehicle_pose.orientation.x,vehicle_pose.orientation.y,vehicle_pose.orientation.z,vehicle_pose.orientation.w])[2]


class WAMV_pub:
    def __init__(self,wamv_name) -> None:
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.left_thrust_ang = 0.0
        self.right_thrust_ang = 0.0
        self.name = wamv_name

    def pub(self,left_thrust_val,right_thrust_val,left_thrust_orient, right_thrust_orient):
        self.left_thrst_pub = rospy.Publisher("/"+self.name+'/thrusters/left_thrust_cmd',Float32,queue_size=10)
        self.right_thrst_pub = rospy.Publisher("/"+self.name+'/thrusters/right_thrust_cmd',Float32,queue_size=10)
        self.left_thrst_ang_pub = rospy.Publisher("/"+self.name+'/thrusters/left_thrust_angle',Float32,queue_size=10)
        self.right_thrst_ang_pub = rospy.Publisher("/"+self.name+'/thrusters/right_thrust_angle',Float32,queue_size=10)

        self.left_thrst_pub.publish(left_thrust_val)
        self.right_thrst_pub.publish(right_thrust_val)
        self.left_thrst_ang_pub.publish(left_thrust_orient)
        self.right_thrst_ang_pub.publish(right_thrust_orient)

mystate = ModelStates()
rospy.init_node('Waypoint_tracking', anonymous=True)

def main():
    ###########################
    # initialise variables - Done
    # identify target waypoint - Done
    # solve the thruster and heading problem(desired heading, the ship shouldn't drift, yaw des = yaw of veh) - Done
    # Call leader model for main leader: Store value in list leader_agent - Done
    # These two are merged just make sure to store value in leader_agent array - Done
    # Use consensus to come up with positions of follower agents: Store value in dict agents - Done
    # Call PID controller on individual agents - Done
    ###########################

    #collect localisation data
    agents = {}
    PID_steer_arr = [PID_controller(math.pi/2,0.01) for i in range(0,9)]
    PID_speed_arr = [PID_controller(100.0,0.01) for i in range(0,9)]
    idx = 0
    wamv_states = retrieve_states() #Returns dictionary of type {"wamv": [pos_x, pos_y, yaw], "wamv2": ... }
    names = list(wamv_states.keys())  #Stores WAMV names : ["wamv","wamv2" ...]
    wamv_speed = [0.0 for i in range(len(names))]
    count = 0
    pos = [] #List of dictionaries for wamv_states
    des_head_arr = []
    curr_head_arr = []
    t_arr = []
    t,t_prev = time.time(), time.time()
    wamv_pub = [WAMV_pub(i) for i in list(wamv_states.keys())]
    leader_pos = [0.0,0.0]
    t_consensus = time.time()
    leader_pos_global = [-870.0,260.0]

    # Initial consensus
    ini_pos = np.mean([[wamv_states[i][0], wamv_states[i][1]] for i in names],axis=0)

    agents_coords,leader_pos = consensus(ini_pos,leader_pos)

    for i in range(len(agents_coords)):
        agents[names[i]] = agents_coords[i]

    t_consenseus = time.time()
    
    while True:
        print(time.time())
        leader_pos_global = leader_model(leader_pos_global,1,0.01,math.atan2(leader_waypoints[idx][1]-leader_pos_global[1], leader_waypoints[idx][0]-leader_pos_global[0]))
        if time.time() - t_consensus >= 1.0:
            ini_pos = np.mean([[wamv_states[i][0], wamv_states[i][1]] for i in names],axis=0)
            del(agents_coords)
            agents_coords,leader_pos = consensus(ini_pos,leader_pos)


            for i in range(len(agents_coords)):
                agents[names[i]] = agents_coords[i]

            t_consenseus = time.time()

        wamv_states = retrieve_states()  

        des_speed = [(1 - 1/(1+3*math.exp(-0.6*abs(i.error)))) for i in PID_steer_arr]
        des_head = [math.atan2(agents[i][1] - wamv_states[i][1], agents[i][0] - wamv_states[i][0]) for i in names]

        curr_head = [wamv_states[i][2] for i in names]

        for i in range(len(names)):
            if len(pos) == 0:
                pos.append(wamv_states)
                continue
            else:
                speed_val = [np.hypot(pos[len(pos)-1][i][1] - wamv_states[i][1], pos[len(pos)-1][i][0] - wamv_states[i][0])/(time.time()-t_prev) for i in names]
                speed_val = [0.0 for i in names]
                print(time.time()-t_prev)
                t_prev = time.time()
        
        pos.append(wamv_states)
        t_arr.append(time.time() - t)

        heading_cmd = [PID_steer_arr[i].PID(0.1, 0 , 0.001, des_head[i],curr_head[i], speed=False,angle=True) for i in range(len(names))]
        speed_cmd   = [PID_speed_arr[i].PID(10.0,10, 0.2,des_speed[i],speed_val[i],speed=True,angle=False) for i in range(len(names))]
        print(speed_cmd)
        for i in range(len(names)):
            agents_coords = list(agents.values())
            agents_xcoords = [agents[i][0] for i in names]
            agents_ycoords = [agents[i][1] for i in names]
            if np.hypot(np.mean(agents_xcoords) - leader_pos_global[0], np.mean(agents_ycoords) - leader_pos_global[1]) <= 1:
                wamv_pub[i].pub(0.0,0.0,0.0,0.0)
                continue
            if abs(PID_steer_arr[i].error) < 0.01:
                wamv_pub[i].pub(speed_cmd[i],speed_cmd[i],0.0,0.0)

            if heading_cmd[i] < 0:
                wamv_pub[i].pub(speed_cmd[i],0.0,-heading_cmd[i],0.0)

            elif heading_cmd[i] > 0:
                wamv_pub[i].pub(0.0,speed_cmd[i],-heading_cmd[i],0.0)
        
        if idx == len(leader_waypoints):
            if np.hypot(np.mean(agents_xcoords) - leader_pos_global[0], np.mean(agents_ycoords) - leader_pos_global[1]) <= 1:
                break
            else:
                continue
        else:
            if np.hypot(leader_pos_global[0] - leader_waypoints[idx][0], leader_pos_global[1] - leader_waypoints[idx][1]) < 5:
                idx += 1
        
        del(agents_coords,agents_xcoords,agents_ycoords)
    
    # plt.plot(pos_x,pos_y)
    # plt.plot([i[0] for i in waypoints],[i[1] for i in waypoints],"*")
    # plt.xlabel("X co-ordinate(m)")
    # plt.ylabel("Y co-ordinate(m)")
    # plt.show()
    # plt.plot(t_arr,des_head_arr)
    # plt.plot(t_arr,curr_head_arr)
    # plt.xlabel("time(s)")
    # plt.ylabel("radians")
    # plt.show()

main()
