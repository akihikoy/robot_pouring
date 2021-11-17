#!/usr/bin/python
from core_tool import *
def Help():
  return '''Template of script.
  Usage: template'''
def Run(ct,*args):
  q1=[0.06289321230331128, -0.12578642460662257, 0.803805932852076, 0.1856116753341626, -0.7133010663668231, -3.103243133892652]
  #q2=[0.06289321230331128, -0.09357282806102411, 0.9280583766708129, 0.1856116753341626, -0.6734175658817965, -3.103243133892652]
  q2=[0.06289321230331128, -0.09357282806102411, 0.9280583766708129, 0.1856116753341626, -0.6734175658817965, -3.103243133892652]
  ct.robot.MoveToQ(q1,1,blocking=True)
  #x1=ct.robot.FK(q1)
  #x2= copy.deepcopy(x1)
  #x2[2]+= 0.02
  for i in range(3):
    #ct.robot.MoveToX(x2,0.5)
    ct.robot.MoveToQ(q2,0.2,blocking=True)
    ct.robot.MoveToQ(q1,0.2,blocking=True)
  print q2
