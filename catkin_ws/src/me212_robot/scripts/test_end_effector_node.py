#!/usr/bin/python
# Publishes a float64multiarray containing [mode, desired_y_e, desired_theta, time] 

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Int16


lower_ye = -0.07
upper_ye = 0.07
theta_carry = 48.01 * np.pi/180;
theta_home = 0
theta_dump = -48.01*np.pi/180;
ee_coord_pub = rospy.Publisher('ee_mode', Float64MultiArray, queue_size=10)
rospy.init_node('ee_mode_pub_sub', anonymous=True)

pos_status = False
mode_data = Float64MultiArray()

exec_mode = 0# Set default mode to 0

y_e = 0
theta = 0
task_time = 0.5

def pos_callback(pos_reached):
    if pos_reached:
        mode = exec_mode
        global y_e, theta, task_time
        if mode == 1: # Home position
            y_e = lower_ye
            theta = theta_home
            task_time = 1
        elif mode == 2: # Excavate
            #mode = 2
            y_e = lower_ye
            theta = theta_carry
            task_time = 1
        elif mode == 3: # Straigh line up
            #mode = 3
            y_e = upper_ye
            theta = theta_carry
            task_time = 2
        elif mode ==4: # Dump
            #mode = 4
            y_e = upper_ye
            theta = theta_dump
            task_time = 1
        else:
            print(type(mode))
            y_e = 0
            theta = 0
            task_time = 0.5 # Doesn't mean anything in this section
        
        mode_data.data = [mode, y_e, theta, task_time]
        rospy.loginfo(mode_data)
        ee_coord_pub.publish(mode_data)

    rospy.loginfo(pos_status)

def mode_callback(mode_num):
    global exec_mode
    exec_mode = mode_num.data
    rospy.loginfo(mode_num)

def main():

    rospy.Subscriber("reached_pos", Bool, pos_callback)
    rospy.Subscriber("exec_mode", Int16, mode_callback)

    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
