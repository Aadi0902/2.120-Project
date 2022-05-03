#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool


PI = 3.1415

'''
Default values:
'''
y_e_up = 0.06
y_e_down = -0.05
y_e_home = 0
y_e_carry = -0.02
theta_up = 0
theta_home = -32.01 * np.pi/180
theta_down = 2*-46.01*np.pi/180

mode_prev = 0
mode = 0

task_time_inp = 0
start_time = 0

mode_pub = rospy.Publisher("exec_mode", Float32MultiArray, queue_size=10) # Publish mode to end effector
vel_curv_pub = rospy.Publisher("vel_curvature", Float32MultiArray, queue_size=10) # Publish veloctiy and curvature to base arduino
rospy.init_node("auto_traj_pub_sub",anonymous=True)

prev_time = rospy.get_time() 
manual_control = False


def manual_callback(man_contr_msg):
    global manual_control
    manual_control = man_contr_msg.data

def path_callback(path_dist):
    global y_e_up, y_e_down, y_e_home, y_e_carry, theta_up, theta_home, theta_down, mode_prev, mode, prev_time, task_time_inp, manual_control
    vel_curv = Float32MultiArray()
    
    rad_turn = 0.25

    d1 = 1
    d2 = rad_turn*PI/2
    d3 = 0.8 - rad_turn
    d4 = 0.8
    d5 = rad_turn*PI/2
    d6 = 0.7
    d7 = 0.2
    d8 = 0.9
    d9 = 0.4
    
    if path_dist.data < d1: # Straight
        mode = 5
        vel_curv.data = [0.5, 0]
    elif path_dist.data < d1+d2: # Turn
        vel_curv.data = [0.5, -1/rad_turn]
    elif path_dist.data < d1+d2+d3: # Straight
        mode = 1
        vel_curv.data = [0.5, 0]
    elif path_dist.data < d1+d2+d3+d4: # Back
        mode = 2
        vel_curv.data = [-0.5, 0]
    elif path_dist.data < d1+d2+d3+d4+d5: # Back turn
        mode = 3
        vel_curv.data = [-0.5, -1/rad_turn]
    elif path_dist.data < d1+d2+d3+d4+d5+d6: # Straight
        vel_curv.data = [0.5, 0]
    elif path_dist.data < d1+d2+d3+d4+d5+d6+d7: # Turn
        vel_curv.data = [0.5, -1/rad_turn]
    elif path_dist.data < d1+d2+d3+d4+d5+d6+d7+d8: # Straight
        vel_curv.data = [0.5, 0]
    elif path_dist.data < d1+d2+d3+d4+d5+d6+d7+d8+d9: # Straight
        mode = 4
        vel_curv.data = [-0.5, 0]
    else:
        mode = 5
        vel_curv.data = [0, 0]
    

    
    if mode == 1: # Down position
        y_e_inp = y_e_down
        theta_inp = theta_home
        task_time_inp = 1
    elif mode == 2: # Excavate
        #mode = 2
        y_e_inp = y_e_carry
        theta_inp = theta_up
        task_time_inp = 1
    elif mode == 3: # Straigh line up
        #mode = 3
        y_e_inp = y_e_up
        theta_inp = theta_up
        task_time_inp = 1.5
    elif mode ==4: # Dump
        #mode = 4
        y_e_inp = y_e_up
        theta_inp = theta_down
        task_time_inp = 1
    else:
        y_e_inp = y_e_home
        theta_inp = theta_up
        task_time_inp = 1 # Doesn't mean anything in this section
        
    
    
    if not mode == mode_prev:
        mode_prev = mode
        prev_time = rospy.get_time()
    
    cur_time = rospy.get_time()
    if cur_time - prev_time < task_time_inp:
        vel_curv.data = [0, 0]
    
    mode_param = Float32MultiArray()
    mode_param.data = [mode, y_e_inp, theta_inp, task_time_inp]

    if not manual_control:
        mode_pub.publish(mode_param)
        vel_curv_pub.publish(vel_curv)
        rospy.loginfo(path_dist.data)
    
def main():
    rospy.Subscriber("path_distance", Float32, path_callback)
    rospy.Subscriber("manual_control", Bool, manual_callback)
    rospy.spin()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
