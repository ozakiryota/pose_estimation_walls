#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf
import numpy as np
import matplotlib.pyplot as plt
import time

start_time = 0.0
t_ = 0.0
z_ = 0.0

def callback(msg):
    # print('callback')

    global start_time
    global t_
    global z_

    if start_time>0:
        t_ = time.time() - start_time
        z_ = msg.pose.pose.position.z

def graph():
    print('graph_odom')
    rospy.init_node('graph_odom', anonymous=True)
    rospy.Subscriber("/imu_odometry", Odometry, callback)
    
    t = [0 for i in range(100)]
    z = [0 for k in range(100)]
    
    plt.ion()
    plt.figure()

    ### z ###
    # plt.subplot(2, 1, 2)
    plt.title("z")
    plt.xlabel("time[s]")
    plt.ylabel("z[m]")
    plt.ylim(-1, 1)
    plt.grid(True)
    li_p, = plt.plot(t, z)
    
    global start_time
    global t_
    global z_

    start_time = time.time()

    while not rospy.is_shutdown():
        # print('loop')
        
        t.append(t_)
        t.pop(0)
        z.append(z_)
        z.pop(0)

        ### z ###
        # plt.subplot(2,1,2)
        li_p.set_xdata(t)
        li_p.set_ydata(z)
        plt.xlim(min(t), max(t))

        plt.draw()
        plt.pause(0.1)
    rospy.spin()

if __name__ == '__main__':
    graph()
