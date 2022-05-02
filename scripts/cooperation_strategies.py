# from _typeshed import Self
from turtle import radians
import time
import numpy as np
import math
import rospy 
import random
import matplotlib.pyplot as plt
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32

def leader_model(prev_pos,v,deltaT,theta_m):
    curr_pos = prev_pos
    curr_pos[0] = prev_pos[0] + (v)*deltaT*math.cos(theta_m)
    curr_pos[1] = prev_pos[1] + (v)*deltaT*math.sin(theta_m)
    return curr_pos

def consensus(ini_pos,prev_leader_pos):
    a = [[0,1,1,1,1,1,1,1,1],
        [1,0,1,1,1,1,1,1,1],
        [1,1,0,1,1,1,1,1,1],
        [1,1,1,0,1,1,1,1,1],
        [1,1,1,1,0,1,1,1,1],
        [1,1,1,1,1,0,1,1,1],
        [1,1,1,1,1,1,0,1,1],
        [1,1,1,1,1,1,1,0,1],
        [1,1,1,1,1,1,1,1,0]]

    A = np.array(a,dtype=Float32)

    # d = np.array([[-1 - (-1/(2*math.sqrt(2))),-1 - (-1/(2*math.sqrt(2)))],[1 - (1/(2*math.sqrt(2))),1 - (1/(2*math.sqrt(2)))]])
    r = 30.0
    r1 = 10.0
    count = 0

    d = np.array([[count+r*math.cos(math.radians(1*60)),count+r*math.sin(math.radians(1*60))],
                  [count+r*math.cos(math.radians(120)),count+r*math.sin(math.radians(120))],
                  [count+r1*math.cos(math.radians(90)),count+r1*math.sin(math.radians(90))],
                  [count+r*math.cos(math.radians(180)),count+r*math.sin(math.radians(180))],
                  [count+r*math.cos(math.radians(240)),count+r*math.sin(math.radians(240))],
                  [count+r1*math.cos(math.radians(210)),count+r1*math.sin(math.radians(210))],
                  [count+r*math.cos(math.radians(300)),count+r*math.sin(math.radians(300))],
                  [count+r*math.cos(math.radians(360)),count+r*math.sin(math.radians(360))],
                  [count+r1*math.cos(math.radians(330)),count+r1*math.sin(math.radians(330))]])

    x = np.array([[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0]])
    buffer = []
    delta_t = 0.01
    arr1 = prev_leader_pos
    for _ in range(0,1000):
        kk = []
        for i in range(0,len(A)):
            sum_1 = np.zeros_like(x[i],dtype = float)
            sum_2 = np.zeros_like(x[i],dtype = float)
            for j in range(0,len(A[0])):
                sum_1 += A[i][j]*(x[i] - x[j] - (d[i] - d[j]))
            kk.append(-1*sum_1)

        # print("X: ", x, "\n", "kk:", kk, "\n")
        x[0] = x[0] + delta_t*kk[0]
        x[1] = x[1] + delta_t*kk[1]
        x[2] = x[2] + delta_t*kk[2]
        x[3] = x[3] + delta_t*kk[3]
        x[4] = x[4] + delta_t*kk[4]
        x[5] = x[5] + delta_t*kk[5]
        x[6] = x[6] + delta_t*kk[6]
        x[7] = x[7] + delta_t*kk[7]
        x[8] = x[8] + delta_t*kk[8]
        buffer.append(np.array([x[0], x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8]]))
        # buffer.append(np.array([x[0], x[1]]))


    for _ in range(0,200):
        kk = []
        arr1 = leader_model(prev_leader_pos,1,delta_t,math.pi/3)
        for i in range(0,len(A)):
            sum_1 = np.zeros_like(x[i],dtype = float)
            sum_2 = np.zeros_like(x[i],dtype = float)
            for j in range(0,len(A[0])):
                sum_1 += A[i][j]*(x[i] - x[j] - (d[i] - d[j]))
            kk.append(-1*sum_1 + arr1)

        # print("X: ", x, "\n", "kk:", kk, "\n")
        x[0] = x[0] + delta_t*kk[0]
        x[1] = x[1] + delta_t*kk[1]
        x[2] = x[2] + delta_t*kk[2]
        x[3] = x[3] + delta_t*kk[3]
        x[4] = x[4] + delta_t*kk[4]
        x[5] = x[5] + delta_t*kk[5]
        x[6] = x[6] + delta_t*kk[6]
        x[7] = x[7] + delta_t*kk[7]
        x[8] = x[8] + delta_t*kk[8]
        buffer.append(np.array([x[0], x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8]]))

    # Transform code 
    for i in range(len(buffer)):
        for j in range(len(buffer[0])):
            buffer[i][j][0] += ini_pos[0]
            buffer[i][j][1] += ini_pos[1]



    consensus_op = [buffer[len(buffer)-1][j] for j in range(0,9)]

    buff_1_x = [buffer[i][0][0] for i in range(len(buffer))] 
    buff_1_y = [buffer[i][0][1] for i in range(len(buffer))] 
    buff_2_x = [buffer[i][1][0] for i in range(len(buffer))] 
    buff_2_y = [buffer[i][1][1] for i in range(len(buffer))] 
    buff_3_x = [buffer[i][2][0] for i in range(len(buffer))] 
    buff_3_y = [buffer[i][2][1] for i in range(len(buffer))] 
    buff_4_x = [buffer[i][3][0] for i in range(len(buffer))] 
    buff_4_y = [buffer[i][3][1] for i in range(len(buffer))] 
    buff_5_x = [buffer[i][4][0] for i in range(len(buffer))] 
    buff_5_y = [buffer[i][4][1] for i in range(len(buffer))] 
    buff_6_x = [buffer[i][5][0] for i in range(len(buffer))] 
    buff_6_y = [buffer[i][5][1] for i in range(len(buffer))] 
    buff_7_x = [buffer[i][6][0] for i in range(len(buffer))] 
    buff_7_y = [buffer[i][6][1] for i in range(len(buffer))] 
    buff_8_x = [buffer[i][7][0] for i in range(len(buffer))] 
    buff_8_y = [buffer[i][7][1] for i in range(len(buffer))] 
    buff_9_x = [buffer[i][8][0] for i in range(len(buffer))] 
    buff_9_y = [buffer[i][8][1] for i in range(len(buffer))] 


    plt.plot(buff_1_x,buff_1_y,)
    plt.plot(buff_2_x,buff_2_y,)
    plt.plot(buff_3_x,buff_3_y,)
    plt.plot(buff_4_x,buff_4_y,)
    plt.plot(buff_5_x,buff_5_y,)
    plt.plot(buff_6_x,buff_6_y,)
    plt.plot(buff_7_x,buff_7_y,)
    plt.plot(buff_8_x,buff_8_y,)
    plt.plot(buff_9_x,buff_9_y,)


    # plt.plot(buff_1_x[len(buff_1_x)-1],buff_1_y[len(buff_1_y)-1],"*")
    # plt.plot(buff_2_x[len(buff_2_x)-1],buff_2_y[len(buff_2_y)-1],"*")
    # plt.plot(buff_3_x[len(buff_1_x)-1],buff_3_y[len(buff_1_x)-1],"*")
    # plt.plot(buff_4_x[len(buff_1_x)-1],buff_4_y[len(buff_1_x)-1],"*")
    # plt.plot(buff_5_x[len(buff_1_x)-1],buff_5_y[len(buff_1_x)-1],"*")
    # plt.plot(buff_6_x[len(buff_1_x)-1],buff_6_y[len(buff_1_x)-1],"*")
    # plt.plot(buff_7_x[len(buff_1_x)-1],buff_7_y[len(buff_1_x)-1],"*")
    # plt.plot(buff_8_x[len(buff_1_x)-1],buff_8_y[len(buff_1_x)-1],"*")
    # plt.plot(buff_9_x[len(buff_1_x)-1],buff_9_y[len(buff_1_x)-1],"*")
    # plt.show()
    return consensus_op,arr1


# arr = [0.0,0.0]
# for i in range(0,10):
#     pos = np.array([-880.0,250.0])
#     _,arr = consensus(pos,arr)