#!/usr/bin/python
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
'''
Default values:
'''
y_e_up = -0.07
y_e_down = 0.07
y_e_home = 0
theta_up = 48.01 * np.pi/180;
theta_home = 0
theta_down = -48.01*np.pi/180;




def main():
    mode_pub = rospy.Publisher('exec_mode', Float64MultiArray, queue_size=10)
    rospy.init_node('user_mode_pub',anonymous=True)

    while not rospy.is_shutdown():
        try:
            mode_inp = int(raw_input("Enter desired end effector mode: "))
        except ValueError:
            print("INVALID INPUT, should be an integer")
            continue
        if mode_inp < 1 or mode_inp > 4:
            print("INVALID Mode input, should be an integer between 1 and 4")
            continue
        
        # Comment following seciton and uncomment next section for having user input instead
        if mode_inp == 1: # Down position
            y_e_inp = y_e_down
            theta_inp = theta_home
            task_time_inp = 1
        elif mode_inp == 2: # Excavate
            #mode = 2
            y_e_inp = theta_down
            theta_inp = theta_up
            task_time_inp = 1
        elif mode_inp == 3: # Straigh line up
            #mode = 3
            y_e_inp = y_e_up
            theta_inp = theta_up
            task_time_inp = 2
        elif mode_inp ==4: # Dump
            #mode = 4
            y_e_inp = y_e_up
            theta_inp = theta_down
            task_time_inp = 1
        else:
            y_e_inp = 0
            theta_inp = 0
            task_time_inp = 0.5 # Doesn't mean anything in this section

        '''
        try:
            y_e_inp = float(raw_input("Enter desired end effector y position (m) : "))
        except ValueError:
            print("INVALID INPUT")
            continue
        try:
            theta_inp = float(raw_input("Enter desired end effector orientation (degrees): "))
            theta_inp = theta_inp * np.pi/180
        except ValueError:
            print("INVALID INPUT")
            continue
        try:
            task_time_inp = float(raw_input("Enter desired task time (s): "))
        except ValueError:
            print("INVALID INPUT")
            continue
        if task_time_inp < 0:
             print("INVALID INPUT, time should be positive")
             continue
        '''
        mode_param = Float64MultiArray()
        mode_param.data = [mode_inp, y_e_inp, theta_inp, task_time_inp]

        mode_pub.publish(mode_param)
        rospy.loginfo(mode_param)
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    