#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   main.py
@Time    :   2023/08/18 16:30:44
@Author  :   by 204 
'''

import rospy
from two_ur.msg import _Robotiq2FGripper_robot_output  as outputMsg # robotic iq
import socket
import numpy as np
import threading
import struct
from geometry_msgs.msg import Wrench
import pandas as pd

pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
command = outputMsg.Robotiq2FGripper_robot_output()

HOST= "192.168.1.113"
PORT = 30003  # socket 
s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.connect((HOST,PORT))

def clows(d):
    
    command.rPR = d
    pub.publish(command)
    rospy.sleep(0.1)

def reset():
    command.rACT = 0
    pub.publish(command)
    rospy.sleep(0.1)     
        
def active():        
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255
    command.rFR  = 150 
    pub.publish(command)
    rospy.sleep(0.1)  

def move(target, way = 1, method = 'j', a = 0.1, v = 0.1):    # 1 is target(x, y, z, r, p, q);  0 is (deta x, deta y, deta z);  method 'j' is movej
    
    th =  0.01
    if(way == 1):
        [mx, my, mz, rx, ry, rz] = get_pos()
        target =  target
        deta = [target[0]-mx, target[1]-my, target[2]-mz, target[3]-rx, target[4]-ry, target[5]-rz]
    else:
        deta = target
        target +=  get_pos()
    if(method=='j'):
        urscript = "movej(p[{}, {}, {}, {}, {}, {}],a={},v={})\n".format(target[0], target[1], target[2], target[3], target[4], target[5], a, v)
    else:
        urscript = "movel(p[{}, {}, {}, {}, {}, {}],a={},v={})\n".format(target[0], target[1], target[2], target[3], target[4], target[5], a, v)         
        
    s.send(urscript.encode('utf8'))
    init_time = rospy.Time.now().to_sec()
    
    # 等待运动到指定位置
    current_pos = get_pos()
    deta = list(map(abs,deta))
    th_p = deta.index(max(deta[0:3]))
    max_p = deta[th_p]
    th_r = deta.index(max(deta[3:]))
    max_r = deta[th_r]
    while(abs(target[th_p]-current_pos[th_p])>max_p*0.1 and abs(target[th_r]-current_pos[th_r])>max_r*0.1):   #             
        current_pos = get_pos()
        if(rospy.Time.now().to_sec()-init_time >= 2):
            break
    
def get_pos():
    data = s.recv(1220)  # 包的长度
    mx = struct.unpack('!d', data[444:452])[0]
    my = struct.unpack('!d', data[452:460])[0]
    mz = struct.unpack('!d', data[460:468])[0]
    rx = struct.unpack('!d', data[468:476])[0]
    ry = struct.unpack('!d', data[476:484])[0]
    rz = struct.unpack('!d', data[484:492])[0]    
    return [mx, my, mz, rx, ry, rz] 

def test_move():
    
    # print(get_pos())
    move([-0.31189620281985164, -0.32731594037888057, 0.4603365375522951, -1.0730957358001838, -2.9260719103476, 0.13018427748253608])
   
def main():

    # active()
    # rospy.sleep(0.5)
    # clows(125)        # 0-255 0 is open max
    test_move()

if __name__ == '__main__':
    
    rospy.init_node('main')
    main()
