#!/usr/bin/env python3

class InverseKinematics:
    def __init__(self,joints_size) -> None:
        self._joints_size = joints_size
        
    def solve(self, xyz, rpy):
        #Inverse kinematics
        qs = [0.0, 0.0, 0.0 , 0.0]
        return qs