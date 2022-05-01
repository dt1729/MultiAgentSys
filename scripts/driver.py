from re import L
import time
from markupsafe import string
import numpy as np
import math
import rospy
import tf_conversions
from std_msgs.msg import Float32
from gazebo_msgs.msg import ModelStates, LinkStates
from matplotlib import pyplot as plt
from controllers import PID_controller, state
#Circular waypoints
waypoint = [-500.0,290]
waypoints = [[waypoint[0] + 10*math.cos(i), waypoint[1] + 10*math.sin(i)] for i in np.arange(0,2*math.pi,0.5)]

#straight Line waypoints

# waypoint = [[-500.0,290],[-460.0,320],[-540,300]]
# waypoints = []
# for j in range(1,len(waypoint)):
#     for i in np.arange(0,1,0.1):
#         waypoints.append([(1-i)*waypoint[j-1][0] + (i)*waypoint[j][0], (1-i)*waypoint[j-1][1] + (i)*waypoint[j][1]])

class WAMV_pub:
    def __init__(self,wamv_name) -> None:
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.left_thrust_ang = 0.0
        self.right_thrust_ang = 0.0
        self.name = wamv_name

    def pub(self,left_thrust_val,right_thrust_val,left_thrust_orient, right_thrust_orient):
        left_thrst_pub = rospy.Publisher("/"+self.name+'/thrusters/left_thrust_cmd',Float32,queue_size=10)
        right_thrst_pub = rospy.Publisher("/"+self.name+'/thrusters/right_thrust_cmd',Float32,queue_size=10)
        left_thrst_ang_pub = rospy.Publisher("/"+self.name+'/thrusters/left_thrust_angle',Float32,queue_size=10)
        right_thrst_ang_pub = rospy.Publisher("/"+self.name+'/thrusters/right_thrust_angle',Float32,queue_size=10)

        left_thrst_pub.publish(left_thrust_val)
        right_thrst_pub.publish(right_thrust_val)
        left_thrst_ang_pub.publish(left_thrust_orient)
        right_thrst_ang_pub.publish(right_thrust_orient)

mystate = ModelStates()
rospy.init_node('Waypoint_tracking', anonymous=True)

def main():
    ###########################
    #
    # initialise variables 
    # identify target waypoint
    # solve the thruster and heading problem(desired heading, the ship shouldn't drift, yaw des = yaw of veh)
    # 
    ###########################

    #collect localisation data


    PID_steer = PID_controller(math.pi/2,0.01)
    PID_speed = PID_controller(100.0,0.01)
    idx = 0
    a = rospy.wait_for_message("/gazebo/link_states",LinkStates)
    wamv_head = tf_conversions.transformations.euler_from_quaternion([a.pose[len(a.pose)-7].orientation.x,a.pose[len(a.pose)-7].orientation.y,a.pose[len(a.pose)-7].orientation.z,a.pose[len(a.pose)-7].orientation.w])[2]
    WAMV_pos = [a.pose[len(a.pose)-7].position.x, a.pose[len(a.pose)-7].position.y, wamv_head] 
    count = 0
    pos_x = []
    pos_y = []
    des_head_arr = []
    curr_head_arr = []
    t_arr = []
    t,t_prev = time.time(), time.time()
    wamv2_pub =WAMV_pub("wamv2")

    while idx < len(waypoints):
        a = rospy.wait_for_message("/gazebo/link_states",LinkStates)        
        if idx == len(waypoints)-1:
            wamv_head = tf_conversions.transformations.euler_from_quaternion([a.pose[len(a.pose)-7].orientation.x,a.pose[len(a.pose)-7].orientation.y,a.pose[len(a.pose)-7].orientation.z,a.pose[len(a.pose)-7].orientation.w])[2]
            WAMV_pos = [a.pose[len(a.pose)-7].position.x, a.pose[len(a.pose)-7].position.y, wamv_head] 
        
            des_speed = (1 - 1/(1+3*math.exp(-0.6*abs(PID_steer.error))))

            des_head = math.atan2(waypoints[idx][1] - WAMV_pos[1], waypoints[idx][0] - WAMV_pos[0])
            curr_head = WAMV_pos[2]
            if len(pos_y) == 0:
                speed_val = 0.0
            else:
                speed_val = np.hypot(pos_y[len(pos_y)-1] - WAMV_pos[1], pos_x[len(pos_x)-1] - WAMV_pos[0])/(time.time()-t_prev)
                t_prev = time.time()

            pos_x.append(WAMV_pos[0])
            pos_y.append(WAMV_pos[1])
            des_head_arr.append(des_head)
            curr_head_arr.append(curr_head)
            t_arr.append(time.time() - t)

            heading_cmd = PID_steer.PID(0.1, 0.000, 0.001, des_head,curr_head, speed=False,angle=True)
            speed_cmd = PID_speed.PID(10.0,10,0.2,des_speed,speed_val,speed=True,angle=False)
            print("\nSteer PID:\t",PID_steer.error,curr_head,"\n")
            print("Speed PID:\t",PID_speed.error,speed_cmd,"\n")
            print("WAMV Pos:\t",WAMV_pos[0], WAMV_pos[1], "\n")
            print("WAMV heading:\t",des_head, wamv_head, "\n")
            print("WAMV Speed:\t",des_speed, speed_val, "\n")
            print("idx\n", idx, "\n")

            if abs(PID_steer.error) < 0.01:
                wamv2_pub.pub(speed_cmd,speed_cmd,0.0,0.0)

            if heading_cmd < 0:
                wamv2_pub.pub(speed_cmd,0.0,-heading_cmd,0.0)

            elif heading_cmd > 0:
                wamv2_pub.pub(0.0,speed_cmd,-heading_cmd,0.0)


        wamv_head = tf_conversions.transformations.euler_from_quaternion([a.pose[len(a.pose)-7].orientation.x,a.pose[len(a.pose)-7].orientation.y,a.pose[len(a.pose)-7].orientation.z,a.pose[len(a.pose)-7].orientation.w])[2]
        WAMV_pos = [a.pose[len(a.pose)-7].position.x, a.pose[len(a.pose)-7].position.y, wamv_head] 
    
        des_speed = (1 - 1/(1+3*math.exp(-0.6*abs(PID_steer.error))))

        des_head = math.atan2(waypoints[idx][1] - WAMV_pos[1], waypoints[idx][0] - WAMV_pos[0])
        curr_head = WAMV_pos[2]
        if len(pos_y) == 0:
            speed_val = 0.0
        else:
            speed_val = np.hypot(pos_y[len(pos_y)-1] - WAMV_pos[1], pos_x[len(pos_x)-1] - WAMV_pos[0])/(time.time()-t_prev)
            t_prev = time.time()

        pos_x.append(WAMV_pos[0])
        pos_y.append(WAMV_pos[1])
        des_head_arr.append(des_head)
        curr_head_arr.append(curr_head)
        t_arr.append(time.time() - t)

        heading_cmd = PID_steer.PID(0.1, 0.000, 0.001, des_head,curr_head, speed=False,angle=True)

        speed_cmd = PID_speed.PID(10.0,10,0.2,des_speed,speed_val,speed=True,angle=False)
        print("\nSteer PID:\t",PID_steer.error,curr_head,"\n")
        print("Speed PID:\t",PID_speed.error,speed_cmd,"\n")
        print("WAMV Pos:\t",WAMV_pos[0], WAMV_pos[1], "\n")
        print("WAMV heading:\t",des_head, wamv_head, "\n")
        print("WAMV Speed:\t",des_speed, speed_val, "\n")
        print("idx\n", idx, "\n")

        norm_steer_error = abs(PID_steer.error)/math.pi

        
        if abs(PID_steer.error) < 0.01:
            wamv2_pub.pub(speed_cmd,speed_cmd,0.0,0.0)

        if heading_cmd < 0:
            wamv2_pub.pub(speed_cmd,0.0,-heading_cmd,0.0)

        elif heading_cmd > 0:
            wamv2_pub.pub(0.0,speed_cmd,-heading_cmd,0.0)

        if np.hypot(WAMV_pos[0] - waypoints[idx][0], WAMV_pos[1] - waypoints[idx][1]) < 5:
            idx += 1
    
    plt.plot(pos_x,pos_y)
    plt.plot([i[0] for i in waypoints],[i[1] for i in waypoints],"*")
    plt.xlabel("X co-ordinate(m)")
    plt.ylabel("Y co-ordinate(m)")
    plt.show()
    plt.plot(t_arr,des_head_arr)
    plt.plot(t_arr,curr_head_arr)
    plt.xlabel("time(s)")
    plt.ylabel("radians")
    plt.show()



main()
