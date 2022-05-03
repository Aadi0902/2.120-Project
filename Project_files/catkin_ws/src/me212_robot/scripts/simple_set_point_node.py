#!/usr/bin/python
# Publishes a float32multiarray containing [mode, desired_y_e, desired_theta, time] 

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Int16

# Variable definition
l_1 = 0.09 # Link 1 length
q_1_limit = 75 * np.pi/180 # Joint angle limit for q_1
q_2_limit = 180 * np.pi/180 # Joint angle limit for q_2

# Default values
y_e_up = 0.06
y_e_down = -0.055
y_e_home = 0
y_e_carry = -0.02

theta_up = 0
theta_home = -48.01 * np.pi/180
theta_down = 2 * -46.01*np.pi/180

desired_y_e = 0
desired_theta = 0

in_y_e = 0.0
in_theta = 0.0

set_point_1 = 0.0
set_point_2 = 0.0

set_point_pub = rospy.Publisher("set_point_topic", Float32MultiArray, queue_size=10)
current_pub = rospy.Publisher("current_topic", Float32MultiArray, queue_size=10)
rospy.init_node("set_point_pub_sub", anonymous=True)

prev_mode = -1
mode  = 0# Set default mode to 0
task_time = 0.5

enc_freq = 20 # 20Hz
tstep = 0

cur_time = rospy.get_rostime()
prev_time = cur_time

def enc_callback(enc_current_values):
    q_1 = float(enc_current_values.data[0])
    q_2 = float(enc_current_values.data[1])

    current_arr = Float32MultiArray()
    current_arr.data = [float(enc_current_values.data[2]), float(enc_current_values.data[3])]

    global prev_mode, set_point_1, set_point_2, l_1, tstep, cur_time, prev_time, in_y_e, in_theta, y_e_up, y_e_down, theta_up, theta_down

    if not mode == prev_mode:
        in_y_e = l_1 * np.sin(q_1)
        in_theta = q_1 + q_2
        prev_mode = mode
        tstep = 1

    if mode > 5 or mode < 1:
        y_e = in_y_e
        theta = in_theta
    else:
        y_e = in_y_e - tstep * (in_y_e - desired_y_e)/(task_time * enc_freq)
        theta = in_theta - tstep * (in_theta - desired_theta)/(task_time * enc_freq)

        y_e = max(min(y_e, y_e_up), y_e_down)
        theta = max(min(theta, theta_up), theta_down)

    (set_point_1, set_point_2) = inverse_k(y_e, theta)

    set_point_arr = Float32MultiArray()
    set_point_arr.data = [float(set_point_1), float(set_point_2), mode, y_e, theta]

    set_point_pub.publish(set_point_arr)
    current_pub.publish(current_arr)

    rospy.loginfo(set_point_arr)

    if not tstep > task_time * enc_freq:
        tstep += 1

def mode_callback(mode_param):
    global mode, task_time, desired_y_e, desired_theta, y_e_up, y_e_down, theta_home, theta_up, theta_down
    mode = int(mode_param.data[0])
    desired_y_e = mode_param.data[1]
    desired_theta = mode_param.data[2]
    task_time = mode_param.data[3]

    if mode == 1: # Down position
        y_e_down = desired_y_e
        theta_home = desired_theta
    elif mode == 2: # Excavate
        #mode = 2
        y_e_carry = desired_y_e
        theta_up = desired_theta
    elif mode == 3: # Straigh line up
        #mode = 3
        y_e_up = desired_y_e
        theta_up = desired_theta
    elif mode == 4: # Dump
        #mode = 4
        y_e_up = desired_y_e
        theta_down = desired_theta
    else:
        theta_home = 0#desired_theta
        y_e_home = 0#desired_y_e
    
    rospy.loginfo(mode)

def inverse_k(y_e, theta):
    global set_point_1, set_point_2
    y_e_max = l_1 # Constrain y_e so that unrealistic values are not possible
    y_e_min = -l_1 # Constrain y_e so that unrealistic values are not possible
    if y_e > y_e_max:
        y_e = y_e_max
    elif y_e < y_e_min:
        y_e = y_e_min
    
    x_e = np.sqrt(l_1*l_1 - y_e*y_e)
    q_1_ik = np.arctan(y_e/x_e)  
    q_2_ik = theta - q_1_ik

    # position limit constraints (update set_point_1 and set_point_2 when they are within the limits)
    if abs(q_1_ik) < q_1_limit and abs(q_2_ik) < q_2_limit:
        set_point_1 = q_1_ik
        set_point_2 = q_2_ik
    else:
        print("Joint limit reached!")
    return (set_point_1, set_point_2)


def main():
    rospy.Subscriber("encoder_current_val", Float32MultiArray, enc_callback)
    rospy.Subscriber("exec_mode", Float32MultiArray, mode_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
