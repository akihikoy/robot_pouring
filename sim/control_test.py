#!/usr/bin/python
# coding: utf-8

from core_tool import *
from copy import deepcopy
import numpy as np


def Help():
    return
  
def Run(ct,*args):
    r = 0.03 # ノズル半径
    # init_rotaion = [0,0.5,0.3,0,-0.58,0] # 初期姿勢
    init_rotaion = [0,0.5,-0.3,0,-0.2,0] # 初期姿勢
    
    ct.robot.MoveToQ(q_trg = init_rotaion, dt = 0.5, blocking = True)
    init_position = deepcopy(ct.robot.FK())
    init_position[1] -= 0.06
    ct.robot.MoveToX(init_position, 0.5, blocking=True)
    
    for theta in np.linspace(0,3,60):
        x = deepcopy(ct.robot.FK())
        x[1] = init_position[1] + r - r*np.cos(theta)
        x[2] = init_position[2] + r*np.sin(theta)
        x[3] = np.sin(-theta/2)
        x[4] = 0
        x[5] = 0
        x[6] = np.cos(-theta/2)
        ct.robot.MoveToX(x, 0.1, blocking=True)