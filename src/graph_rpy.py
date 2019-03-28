#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf
import numpy as np
import matplotlib.pyplot as plt
import time

start_time = 0.0
t_ = 0.0
roll_ = 0.0
pitch_ = 0.0

def callback(msg):
    # print('callback')

    global start_time
    global t_
    global roll_
    global pitch_
    
    e = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

    if start_time>0:
        t_ = time.time() - start_time
        roll_ = e[0]
        pitch_ = e[1]

def graph():
    print('graph_rpy')
    rospy.init_node('graph_rpy', anonymous=True)
    rospy.Subscriber("/imu_odometry", Odometry, callback)
    
    t = [0 for i in range(50)]
    roll = [0 for j in range(50)]
    pitch = [0 for k in range(50)]
    
    plt.ion()
    plt.figure()

    ### roll ###
    plt.subplot(2, 1, 1)
    # plt.title("roll")
    plt.xlabel("time[s]")
    plt.ylabel("roll[deg]")
    plt.ylim(-5, 5)
    plt.grid(True)
    li_r, = plt.plot(t, roll)

    ### pitch ###
    plt.subplot(2, 1, 2)
    # plt.title("pitch")
    plt.xlabel("time[s]")
    plt.ylabel("pitch[deg]")
    plt.ylim(-5, 5)
    plt.grid(True)
    li_p, = plt.plot(t, pitch)
    
    global start_time
    global t_
    global roll_
    global pitch_

    start_time = time.time()

    while not rospy.is_shutdown():
        # print('loop')
        
        t.append(t_)
        t.pop(0)
        roll.append(roll_/np.pi*180.0)
        roll.pop(0)
        pitch.append(pitch_/np.pi*180.0)
        pitch.pop(0)
        
        ### roll ###
        plt.subplot(2,1,1)
        li_r.set_xdata(t)
        li_r.set_ydata(roll)
        plt.xlim(min(t), max(t))
        # plt.draw()

        ### roll ###
        plt.subplot(2,1,2)
        li_p.set_xdata(t)
        li_p.set_ydata(pitch)
        plt.xlim(min(t), max(t))

        plt.draw()
        plt.pause(0.1)
    rospy.spin()

if __name__ == '__main__':
    graph()
