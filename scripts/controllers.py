from _typeshed import Self
import numpy as np
import rospy 
import maplotlib.pyplot as plt

class state:
    def __init__(self) -> None:
        self.x = 0
        self.y = 0 
        self.phi = 0
        self.v = 0

class PID_controller:
    def __init__(self,sat,simTStep) -> None:
        self.output = 0
        self.error = 0
        self.error_sum = 0
        self.saturation = sat
        self.dt = simTStep

    def PID(self,Kp, Ki, Kd, state_val, curr_val,speed=True,angle=False) -> float:
        curr_error = state_val - curr_val
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
                
        return a