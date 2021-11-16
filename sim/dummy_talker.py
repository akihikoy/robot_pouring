#!/usr/bin/python
# coding: utf-8

from core_tool import *
import std_msgs
from time import time


def Help():
    pass


def Run(ct, *args):
    interval = 5
    
    # ct.AddPub('weight_raw', '/weight/raw', ay_util_msgs.msg.StringStamped)
    ct.AddPub('weight_value', '/weight/value', std_msgs.msg.Float64)
    
    # total_amount = 0    
    # damount_list = [0.0,0.01,0.0,-1]
    total_amount_list = [0,1,2,3]
    
    for total_amount in total_amount_list:
        t0 = time()
        t1 = t0
        while (t1 - t0) < interval:
            # total_amount += damount*1e-3
            print('weight: {}'.format(total_amount))
            
            ct.pub.weight_value.publish(total_amount)
    
            t1 = time()

    # ct.DelPub('weight_raw')
    ct.DelPub('weight_value')