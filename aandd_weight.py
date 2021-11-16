#!/usr/bin/python
from core_tool import *
roslib.load_manifest('ay_util_msgs')
import ay_util_msgs.msg

def Help():
  return '''A and D weight scale utility.
  ay_util AandDEW_weight.py should be launched beforehand.
  Usage:
    aandd_weight 'on' [, NODE]
    aandd_weight 'setup' [, NODE]
      Start to subscribe the topics.
      The observed data can be accessed via: ct.GetAttr(TMP,NODE)
      NODE: Node name of the weight scale; default:weight.
    aandd_weight
    aandd_weight 'off' [, NODE]
    aandd_weight 'clear' [, NODE]
      Stop to subscribe the topics.
      NODE: Node name of the weight scale; default:weight.
  '''

def ReceiveRaw(ct,l,msg):
  l.raw= msg.data
  l.stamp_raw= msg.header
  if ct.callback.weight is not None:
    ct.callback.weight('raw',l)

def ReceiveValue(ct,l,msg):
  l.value= msg.data
  l.stamp_value= msg.header
  if ct.callback.weight is not None:
    ct.callback.weight('value',l)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  def UnSubscribe(node):
    ct.DelSub('{node}_raw'.format(node=node))
    ct.DelSub('{node}_value'.format(node=node))

  if command in ('on','setup'):
    node= args[0] if len(args)>0 else 'weight'

    ct.SetAttr(TMP,node, TContainer())
    l= ct.GetAttr(TMP,node)
    l.node= node
    l.value= None  #Weight in gram.
    l.stamp_value= None  #Stamp at the weight observation.
    l.raw= None  #Raw reading.
    l.stamp_raw= None  #Stamp at the raw observation.
    ct.callback.weight= None

    ct.AddSub('{node}_raw'.format(node=node), '/{node}/raw'.format(node=node), ay_util_msgs.msg.StringStamped, lambda msg,ct=ct,l=l:ReceiveRaw(ct,l,msg))
    ct.AddSub('{node}_value'.format(node=node), '/{node}/value'.format(node=node), ay_util_msgs.msg.Float64Stamped, lambda msg,ct=ct,l=l:ReceiveValue(ct,l,msg))

  elif command in ('off','clear'):
    node= args[0] if len(args)>0 else 'weight'
    UnSubscribe(node)

