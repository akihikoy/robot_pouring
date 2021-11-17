#!/usr/bin/python
# coding: utf-8

from core_tool import *
from copy import deepcopy
import numpy as np
import std_msgs
import ay_util_msgs.msg
from sensor_msgs.msg import JointState
from time import time, sleep
import os
from glob import glob
import pandas as pd



def Help():
    return

# def update_damount(damount_list, msg):
#     damount_list[0] = msg.data
    
def update_total_amount(total_amount_list, msg):
    total_amount_list[0] = msg.data
    
def update_joint_state(position_list, velocity_list, msg):
    position_list[0] = msg.position
    velocity_list[0] = msg.velocity
    
def init_process():
    pass
    
def terminal_process(ct, theta):
    x = deepcopy(ct.robot.FK())
    
    q1 = deepcopy(x[3:])
    q2 = QFromAxisAngle([1.,0,0],+theta)
    q3 = MultiplyQ(q2,q1)
    x[3] = q3[0]
    x[4] = q3[1]
    x[5] = q3[2]
    x[6] = q3[3]
    
    # x[3] = 0
    # x[4] = 0
    # x[5] = 0
    # x[6] = 1
    ct.robot.MoveToX(x, 1, blocking=True)
    ct.DelSub('weight_value')
    ct.DelSub('joint_states')
    
def report_log(state, total_time, theta, damount, total_amount):
    print('{}, {:.2f}s, {:.1f}rad, {}ml, {}ml'.format(state, total_time, np.rad2deg(theta), damount, total_amount))
    
    
def tip(ct, port, init_position, d, r, dtheta, max_amount = 100, theta_max = 0.9*np.pi, max_dtime = 4, max_time = 60, init_angle = 0):
    R = np.sqrt(d**2 + r**2)
    init_theta = np.arctan(r/d)
    # theta = 0
    theta = init_angle
    dtime = 0
    total_amount_list = [0]
    position_list = [None]
    velocity_list = [None]
    flow_flag = False
    
    ct.robot.MoveToX(init_position, 2, blocking=True)
    
    # theta = np.pi*0.25
    # init_position = [0.24,-0.1,0.07,np.sin(-init_angle/2),0,0,np.cos(-init_angle/2)] # 初期姿勢
    
    # init_position[1] -= r
    # init_position[2] -= d
    init_position2 = deepcopy(init_position)
    init_position2[1] = init_position2[1] + r - R*np.sin(init_theta + init_angle)
    init_position2[2] = init_position2[2] + d - R*np.cos(init_theta + init_angle)
    ct.robot.MoveToX(init_position2, 2, blocking=True)
    
    init_time = time()
    total_time = 0
    flow_start_time = None
    prev_total_amount = 0
    
    print("Start Tipping")
    amounts = []
    times = []
    rads = []
    status = []
    positions = []
    velocities = []
    while True:
        ct.AddSub('weight_value', port, std_msgs.msg.Float64, lambda msg: update_total_amount(total_amount_list, msg))
        ct.AddSub('joint_states', '/joint_states', JointState, lambda msg: update_joint_state(position_list, velocity_list, msg))
        
        total_amount = total_amount_list[0]
        damount = max(total_amount - prev_total_amount, 0)
        total_time = time() - init_time
        
        if total_amount >= max_amount:
            state = 'MAX AMOUNT'
            report_log(state, total_time, theta, damount, total_amount)
            break 
        elif total_time >= max_time:
            state = 'MAX TIME'
            report_log(state, total_time, theta, damount, total_amount)
            break 
        elif theta >= theta_max:
            state = 'MAX THETA'
            report_log(state, total_time, theta, damount, total_amount)
            break
        elif round(damount) == 0 and not flow_flag:
            state = 'ROTATE'
            report_log(state, total_time, theta, damount, total_amount)
            theta += dtheta
            x = deepcopy(ct.robot.FK())
            x[1] = init_position[1] + r - R*np.sin(init_theta + theta)
            x[2] = init_position[2] + d - R*np.cos(init_theta + theta)
            # x[1] = init_position[1] - R*np.sin(init_theta + theta)
            # x[2] = init_position[2] - R*np.cos(init_theta + theta)
            # x[3] = np.sin(-theta/2 )
            # x[4] = 0
            # x[5] = 0
            # x[6] = np.cos(-theta/2 )
            
            # tmp_q = np.outer(x[3:], [np.sin(-theta/2),0,0, np.cos(-theta/2)])
            # x[3] = tmp_q[0]
            # x[4] = tmp_q[1]
            # x[5] = tmp_q[2]
            # x[6] = tmp_q[3]
            
            # q1 = np.quaternion(*x[3:])
            # q2 = np.quaternion(*[np.sin(-theta/2),0,0, np.cos(-theta/2)])
            # q1 = np.quaternion(x[6],x[3],x[4],x[5])
            # q2 = np.quaternion(np.cos(-theta/2),np.sin(-theta/2),0,0)
            # q3 = q1*q2
            # x[3] = q3.x
            # x[4] = q3.y
            # x[5] = q3.z
            # x[6] = q3.w
            
            q1 = deepcopy(init_position[3:])
            q2 = QFromAxisAngle([1.,0,0],-theta)
            q3 = MultiplyQ(q2,q1)
            x[3] = q3[0]
            x[4] = q3[1]
            x[5] = q3[2]
            x[6] = q3[3]
            
            ct.robot.MoveToX(x, 1, blocking=False)
            sleep(0.5)
        elif round(damount) == 0 and flow_flag:
            state = 'KEEP'
            report_log(state, total_time, theta, damount, total_amount)
            dtime = time() - flow_start_time
            if dtime >= max_dtime:
                flow_flag = False
        elif damount > 0:
            state = 'FLOW'
            report_log(state, total_time, theta, damount, total_amount)
            flow_flag = True
            dtime = 0
            flow_start_time = time()
        else:
            state = 'Exception'
            raise(Exception)
            
        prev_total_amount = total_amount
        amounts.append(total_amount)
        times.append(total_time)
        rads.append(theta)
        status.append(state)
        positions.append(position_list[0])
        velocities.append(velocity_list[0])

    print("Finish Tiping")
    terminal_process(ct, theta)
    
    return times, rads, amounts, status, positions, velocities


def shake(ct, port, init_position, d, r, dtheta, max_amount = 100, theta_max = 0.9*np.pi, max_dtime = 4, max_time = 60, init_angle = 0, shake_range = 0.4, shake_angle = np.pi/4, shake_time = 1):
    R = np.sqrt(d**2 + r**2)
    init_theta = np.arctan(r/d)
    theta = init_angle
    dtime = 0
    total_amount_list = [0]
    position_list = [None]
    velocity_list = [None]
    flow_flag = False
    
    # init_position = [0.24,-0.1,0.07,np.sin(-init_angle/2),0,0,np.cos(-init_angle/2)] # 初期姿勢
    
    # init_position[1] -= r
    # init_position[2] -= d
    init_position2 = deepcopy(init_position)
    init_position2[1] = init_position2[1] + r - R*np.sin(init_theta + init_angle)
    init_position2[2] = init_position2[2] + d - R*np.cos(init_theta + init_angle)
    ct.robot.MoveToX(init_position2, 2, blocking=True)
    
    init_time = time()
    total_time = 0
    flow_start_time = None
    prev_total_amount = 0
    
    print("Start Shaking")
    amounts = []
    times = []
    rads = []
    status = []
    positions = []
    velocities = []
    while True:
        ct.AddSub('weight_value', port, std_msgs.msg.Float64, lambda msg: update_total_amount(total_amount_list, msg))
        ct.AddSub('joint_states', '/joint_states', JointState, lambda msg: update_joint_state(position_list, velocity_list, msg))
        
        total_amount = total_amount_list[0]
        damount = max(total_amount - prev_total_amount, 0)
        total_time = time() - init_time
        
        if total_amount >= max_amount:
            state = 'MAX AMOUNT'
            report_log(state, total_time, theta, damount, total_amount)
            break 
        elif total_time >= max_time:
            state = 'MAX TIME'
            report_log(state, total_time, theta, damount, total_amount)
            break 
        elif theta < theta_max:
            state = 'ROTATE'
            report_log(state, total_time, theta, damount, total_amount)
            theta += dtheta
            x = deepcopy(ct.robot.FK())
            x[1] = init_position[1] + r - R*np.sin(init_theta + theta)
            x[2] = init_position[2] + d - R*np.cos(init_theta + theta)
            # # x[1] = init_position[1] - R*np.sin(init_theta + theta)
            # # x[2] = init_position[2] - R*np.cos(init_theta + theta)
            # x[3] = np.sin(-theta/2)
            # x[4] = 0
            # x[5] = 0
            # x[6] = np.cos(-theta/2)
            
            q1 = deepcopy(init_position[3:])
            q2 = QFromAxisAngle([1.,0,0],-theta)
            q3 = MultiplyQ(q2,q1)
            x[3] = q3[0]
            x[4] = q3[1]
            x[5] = q3[2]
            x[6] = q3[3]
            
            ct.robot.MoveToX(x, 1, blocking=False)
            sleep(0.5)
        elif theta >= theta_max:
            state = 'SHAKE'
            report_log(state, total_time, theta, damount, total_amount)
            
            x = deepcopy(ct.robot.FK())
            x[1] = init_position[1] + r - R*np.sin(init_theta + theta_max)
            x[2] = init_position[2] + d - R*np.cos(init_theta + theta_max)
            
            x1 = deepcopy(x)
            x1[1] -= shake_range*np.sin(shake_angle)
            x1[2] += shake_range*np.cos(shake_angle)
            
            x2 = deepcopy(x)
            x2[1] += shake_range*np.sin(shake_angle)
            x2[2] -= shake_range*np.cos(shake_angle)
            
            x_traj=[x1]+XInterpolation(x1, x2, 5)+XInterpolation(x2, x1, 5)
            t_traj=TTrajFromXTraj(x_traj, 0.5, 0.01)
            ct.robot.FollowXTraj(x_traj,t_traj, blocking=True)
            # sleep(t_traj[-1])
            
            # x[1] = init_position[1] + r - R*np.sin(init_theta + theta_max)
            # x[2] = init_position[2] + d - R*np.cos(init_theta + theta_max)
            
            # x_tmp = deepcopy(x)
            # x_tmp[1] -= (shake_range/2)*np.sin(shake_angle)
            # x_tmp[2] += (shake_range/2)*np.cos(shake_angle)
            # ct.robot.MoveToX(x_tmp, shake_time/2, blocking=True)
            # x_tmp = deepcopy(x)
            # x_tmp[1] += (shake_range/2)*np.sin(shake_angle)
            # x_tmp[2] -= (shake_range/2)*np.cos(shake_angle)
            # ct.robot.MoveToX(x_tmp, shake_time, blocking=True)
            # ct.robot.MoveToX(x, shake_time/2, blocking=True)
        else:
            state = 'Exception'
            raise(Exception)
            
        prev_total_amount = total_amount
        amounts.append(total_amount)
        times.append(total_time)
        rads.append(theta)
        status.append(state)
        positions.append(position_list[0])
        velocities.append(velocity_list[0])

    print("Finish Shaking")
    terminal_process(ct, theta)
    
    return times, rads, amounts, status, positions, velocities


def arg_split(string_arg):
    return float(string_arg.split('=')[-1])
  
def Run(ct,*args):
    max_amount = arg_split(args[0])
    theta_max = arg_split(args[1])*np.pi
    max_dtime = arg_split(args[2])
    max_time = arg_split(args[3])
    dtheta = arg_split(args[4])
    
    shake_range = arg_split(args[5])
    shake_angle = arg_split(args[6])*np.pi
    shake_time = arg_split(args[7])
    
    init_angle = arg_split(args[8])*np.pi
    
    r = arg_split(args[9])
    d = arg_split(args[10]) + (0.09-r)
    # skill = 'tip' if args[2] == 0 else 'shake'
    skill = args[11].split('=')[-1]
    
    
    # example
    # mysim.actual_machine.sim.feedbuck_control_test 'max_amount=100', 'theta_max=0.9', 'max_dtime=4', 'max_time=10', 'dtheta=0.04', 'shake_range=0.04', 'shake_angle=0.5', 'shake_time=0.2', 'init_angle=0.25', 'r=0.045', 'd=0.05', 's=shake'
    
    # skill = 'tip' # tip or shake
    # skill = 'shake'
    # r = 0.045 # ノズル半径
    # d = 0.08 # 把持位置から容器口までの高さ
    
    # max_amount = 100
    # theta_max = 0.9*np.pi
    # max_dtime = 4
    # max_time = 10
    # dtheta = 0.04
    
    # shake_range = 0.04
    # shake_angle = 0.5*np.pi
    # shake_time = 0.2
    
    # init_angle = np.pi*0.5
    
    port = '/weight/value'
    
    # ログディレクトリ生成
    # base_logdir = '/home/yashima/ros_ws/ay_tools/ay_skill_extra/mysim/actual_machine/logs/'
    base_logdir = ct.DataBaseDir()+'robot_pour/'
    base_logdir += '{}_r{}_d{}_tm{}_ma{}_md{}_mt{}_dt{}/'.format(skill,r,d,theta_max,max_amount,max_dtime,max_time,dtheta)
    if not os.path.exists(base_logdir):
        os.mkdir(base_logdir)
    n = len(glob(base_logdir+"*"))+1
    logdir = base_logdir + str(n) + '/'
    os.mkdir(logdir)
    
    # 初期姿勢に移動
    # ct.robot.MoveToQ([0.]*ct.robot.DoF(), 2, blocking=True)
    # init_position = [0.22,-0.1,0.1,0,0,0,1] # 初期姿勢    
    # init_position = [0.24,-0.1,0.07,0,0,0,1] # 初期姿勢
    init_position = [0.25, 0.0235149419831686, 0.12064477698830921, -0.4999954388940641, 0.5000045609618852, -0.49998631655460113, 0.5000136831733775]
    
    # メインの処理
    if skill == 'tip':
        times, rads, amounts, status, positions, velocities = tip(ct, port, init_position, d, r, dtheta, max_amount, theta_max, max_dtime, max_time, init_angle)
    elif skill == 'shake':
        times, rads, amounts, status, positions, velocities = shake(ct, port, init_position, d, r, dtheta, max_amount, theta_max, max_dtime, max_time, init_angle, shake_range, shake_angle, shake_time)
    else:
        raise(Exception)
    
    # ログをcsvに保存
    df = pd.DataFrame({'times': times, 'rads': rads, 'amounts': amounts, 'status': status, 'positions': positions, 'velocities': velocities})
    df.to_csv(logdir + 'log.csv')