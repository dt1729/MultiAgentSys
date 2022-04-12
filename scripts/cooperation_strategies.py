# from _typeshed import Self
import tf
import time
import numpy as np
import math
import rospy 
import random
import matplotlib.pyplot as plt
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32


# rospy.init_node("temp")
# a = rospy.wait_for_message("/gazebo/model_states",ModelStates)


def main():
    A = np.array([[0.0, 1.0],[1.0, 0.0]])
    a = [[0,1,1,1,1,1,1,1,1],
        [1,0,1,1,1,1,1,1,1],
        [1,1,0,1,1,1,1,1,1],
        [1,1,1,0,1,1,1,1,1],
        [1,1,1,1,0,1,1,1,1],
        [1,1,1,1,1,0,1,1,1],
        [1,1,1,1,1,1,0,1,1],
        [1,1,1,1,1,1,1,0,1],
        [1,1,1,1,1,1,1,1,0]]

    # A = np.array(a)

    d = np.array([[-1 - (-1/(2*math.sqrt(2))),-1 - (-1/(2*math.sqrt(2)))],[1 - (1/(2*math.sqrt(2))),1 - (1/(2*math.sqrt(2)))]])
    # d = np.array([[0,0],[0,0]])
    # x = np.array([[-1.0,-1.0],[1.0,1.0],[-1.0,1.0],[1.0,-1.0],[0.0,1.0],[1.0,0.0],[0.0,-1.0],[-1.0,0.0],[1.0,2.0]])
    x = np.array([[-1.0,-1.0],[1.0,1.0]])
    buffer = []
    delta_t = 0.01
    for count in range(0,1000):
        kk = []
        for i in range(len(A)):
            sum_1 = np.zeros_like(x[i],dtype = float)
            for j in range(len(A[0])):
                sum_1 += A[i][j]*(x[i] - x[j] - d[i])
            print(sum_1,"\n")
            kk.append(sum_1)

        print("X: ", x, "\n", "kk:", kk, "\n")
        x[0] = x[0] - delta_t*kk[0]
        x[1] = x[1] - delta_t*kk[1]
        # x[2] = x[2] - delta_t*kk[2]
        # x[3] = x[3] - delta_t*kk[3]
        # x[4] = x[4] - delta_t*kk[4]
        # x[5] = x[5] - delta_t*kk[5]
        # x[6] = x[6] - delta_t*kk[6]
        # x[7] = x[7] - delta_t*kk[7]
        # x[8] = x[8] - delta_t*kk[8]
        # buffer.append(np.array([x[0], x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8]]))
        buffer.append(np.array([x[0], x[1]]))

    buff_1_x = [buffer[i][0][0] for i in range(len(buffer))] 
    buff_1_y = [buffer[i][0][1] for i in range(len(buffer))] 
    buff_2_x = [buffer[i][1][0] for i in range(len(buffer))] 
    buff_2_y = [buffer[i][1][1] for i in range(len(buffer))] 
    # buff_3_x = [buffer[i][2][0] for i in range(len(buffer))] 
    # buff_3_y = [buffer[i][2][1] for i in range(len(buffer))] 
    # buff_4_x = [buffer[i][3][0] for i in range(len(buffer))] 
    # buff_4_y = [buffer[i][3][1] for i in range(len(buffer))] 
    # buff_5_x = [buffer[i][4][0] for i in range(len(buffer))] 
    # buff_5_y = [buffer[i][4][1] for i in range(len(buffer))] 
    # buff_6_x = [buffer[i][5][0] for i in range(len(buffer))] 
    # buff_6_y = [buffer[i][5][1] for i in range(len(buffer))] 
    # buff_7_x = [buffer[i][6][0] for i in range(len(buffer))] 
    # buff_7_y = [buffer[i][6][1] for i in range(len(buffer))] 
    # buff_8_x = [buffer[i][7][0] for i in range(len(buffer))] 
    # buff_8_y = [buffer[i][7][1] for i in range(len(buffer))] 
    # buff_9_x = [buffer[i][8][0] for i in range(len(buffer))] 
    # buff_9_y = [buffer[i][8][1] for i in range(len(buffer))] 
    print(x)   
    plt.plot(buff_1_x,buff_1_y,"*")
    plt.plot(buff_2_x,buff_2_y,"*")
    # plt.plot(buff_3_x,buff_3_y,"*")
    # plt.plot(buff_4_x,buff_4_y,"*")
    # plt.plot(buff_5_x,buff_5_y,"*")
    # plt.plot(buff_6_x,buff_6_y,"*")
    # plt.plot(buff_7_x,buff_7_y,"*")
    # plt.plot(buff_8_x,buff_8_y,"*")
    # plt.plot(buff_9_x,buff_9_y,"*")
    plt.show()

main()