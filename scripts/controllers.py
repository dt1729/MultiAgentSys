import math
from pydoc import describe
import numpy as np
import rospy 

class state:
    def __init__(self) -> None:
        self.x = 0
        self.y = 0 
        self.phi = 0
        self.v = 0

class PID_controller:
    def __init__(self,sat,simTStep) -> None:
        self.output = 0.0
        self.error = 0.0
        self.error_sum = 0.0
        self.saturation = sat
        self.dt = simTStep

    def PID(self,Kp, Ki, Kd, des_val, curr_val,speed=True,angle=False) -> float:
        if angle:
            curr_error = des_val - curr_val
            if curr_error > math.pi:
                curr_error = curr_error - 2*math.pi
            elif curr_error < -math.pi:
                curr_error = curr_error + 2*math.pi 
        else:
            curr_error = des_val - curr_val

        self.error = curr_error
        self.error_sum = self.error + curr_error*self.dt
        error_diff = (curr_error - self.error)/self.dt

        a = Kp*self.error + Ki*(self.error_sum) + Kd*error_diff
        if speed:
            if a > self.saturation:
                a = self.saturation
                self.error_sum = 0
            elif a < 0:
                a = 0
                
        if angle:
            if abs(a) > self.saturation:
                self.error_sum = 0
                if a > 0:
                    a = self.saturation
                else:
                    a = -1*self.saturation
                # if self.error < 0.05:
                #     return 0
        if abs(self.error) < 0.2:
            self.error_sum = 0             
        return a