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
theta_up = 5*np.pi/180
theta_home = -42.01 * np.pi/180
theta_down = 2*-52.01*np.pi/180

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
    
    robot_vel = 1
    
    rad_turn = 0.25

    d1 = 0.9
    d2 = rad_turn*PI/2
    d3 = 0.6 - rad_turn
    d4 = 0.312
    d5 = rad_turn*PI/2
    d6 = 1.3
    d7 = 0.2
    d8 = 0.7
    d9 = 0.05#0.4
    
    # To_do
    d_margein = 0.10
    d_turn_depo = rad_turn*PI/180*45
    d_straight_depo = 1.3
    l1 = d1+d2+d3+d4+d5+d6+d7+d8+d9
    l2 = l1+d_turn_depo+d_straight_depo+d2+d3+2*d_margein+d4+d5+d6+d7+d8+d9    
    
    
    if path_dist.data < d1: # Straight
        mode = 5
        vel_curv.data = [robot_vel, 0]
    elif path_dist.data < d1+d2: # Turn
        mode = 1
        vel_curv.data = [robot_vel, -1/rad_turn]
    elif path_dist.data < d1+d2+d3: # Straight
        vel_curv.data = [robot_vel, 0]
    elif path_dist.data < d1+d2+d3+d4: # Back
        mode = 2
        vel_curv.data = [-robot_vel, 0]
    elif path_dist.data < d1+d2+d3+d4+d5: # Back turn
        mode = 3
        vel_curv.data = [-robot_vel, -1/rad_turn]
    elif path_dist.data < d1+d2+d3+d4+d5+d6: # Straight
        vel_curv.data = [robot_vel, 0]
    elif path_dist.data < d1+d2+d3+d4+d5+d6+d7: # Turn
        vel_curv.data = [robot_vel, -1/rad_turn]
    elif path_dist.data < d1+d2+d3+d4+d5+d6+d7+d8: # Straight
        vel_curv.data = [robot_vel, 0]
    elif path_dist.data < d1+d2+d3+d4+d5+d6+d7+d8+d9: # Back
        mode = 4
        vel_curv.data = [-robot_vel, 0]
    # 2nd trip : target R
    # deposit -> center
    elif path_dist.data < l1+d_turn_depo: # Back and turn       #0
        mode = 3 # Raise bucket
        vel_curv.data = [-robot_vel, -1/rad_turn]
    elif path_dist.data < l1+d_turn_depo+d_straight_depo: # back    #1
        vel_curv.data = [-robot_vel, 0]
    elif path_dist.data < l1+d_turn_depo+d_straight_depo+d2: # turn right   #2
        mode = 1 # Heading to excavate
        vel_curv.data = [robot_vel, -1/rad_turn]
    # center -> deposit
    elif path_dist.data < l1+d_turn_depo+d_straight_depo+d2+d3+d_margein: # Straight        #3
        vel_curv.data = [robot_vel, 0]
    elif path_dist.data < l1+d_turn_depo+d_straight_depo+d2+d3+2*d_margein+d4: # Back        #4
        mode = 2 # Excavate
        vel_curv.data = [-robot_vel, 0]
    elif path_dist.data < l1+d_turn_depo+d_straight_depo+d2+d3+2*d_margein+d4+d5: # Back turn        #5
        mode = 3 # Raise bucket
        vel_curv.data = [-robot_vel, -1/rad_turn]
    elif path_dist.data < l1+d_turn_depo+d_straight_depo+d2+d3+2*d_margein+d4+d5+d6: # Straight        #6
        vel_curv.data = [robot_vel, 0]
    elif path_dist.data < l1+d_turn_depo+d_straight_depo+d2+d3+2*d_margein+d4+d5+d6+d7: # Turn        #7
        vel_curv.data = [robot_vel, -1/rad_turn]
    elif path_dist.data < l1+d_turn_depo+d_straight_depo+d2+d3+2*d_margein+d4+d5+d6+d7+d8: # Straight        #8
        vel_curv.data = [robot_vel, 0]
    elif path_dist.data < l1+d_turn_depo+d_straight_depo+d2+d3+2*d_margein+d4+d5+d6+d7+d8+d9: # Straight        #9
        mode = 4 # Deposit
        vel_curv.data = [-robot_vel, 0]
    # 3rd trip : target L
    # deposit -> center
    elif path_dist.data < l2+d_turn_depo: # Back and turn       #0
        mode = 3 # Raise bucket
        vel_curv.data = [-robot_vel, -1/rad_turn]
    elif path_dist.data < l2+d_turn_depo+d_straight_depo: # back    #1
        vel_curv.data = [-robot_vel, 0]
    elif path_dist.data < l2+d_turn_depo+d_straight_depo+d2: # turn left   #2
        mode = 1 # Heading to excavate
        vel_curv.data = [robot_vel, 1/rad_turn]
    # center -> deposit
    elif path_dist.data < l2+d_turn_depo+d_straight_depo+d2+d3: # Straight        #3
        vel_curv.data = [robot_vel, 0]
    elif path_dist.data < l2+d_turn_depo+d_straight_depo+d2+d3+d4: # Back        #4
        mode = 2 # Excavate
        vel_curv.data = [-robot_vel, 0]
    elif path_dist.data < l2+d_turn_depo+d_straight_depo+d2+d3+d4+d5: # Back turn        #5
        mode = 3 # Raise bucket
        vel_curv.data = [-robot_vel, 1/rad_turn]
    elif path_dist.data < l2+d_turn_depo+d_straight_depo+d2+d3+d4+d5+d6: # Straight        #6
        vel_curv.data = [robot_vel, 0]
    elif path_dist.data < l2+d_turn_depo+d_straight_depo+d2+d3+d4+d5+d6+d7: # Turn        #7
        vel_curv.data = [robot_vel, -1/rad_turn]
    elif path_dist.data < l2+d_turn_depo+d_straight_depo+d2+d3+d4+d5+d6+d7+d8: # Straight        #8
        vel_curv.data = [robot_vel, 0]
    elif path_dist.data < l2+d_turn_depo+d_straight_depo+d2+d3+d4+d5+d6+d7+d8+d9: # Straight        #9
        mode = 4 # Deposit
        vel_curv.data = [-robot_vel, 0] 
    #else command
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
        task_time_inp = 2
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
    
