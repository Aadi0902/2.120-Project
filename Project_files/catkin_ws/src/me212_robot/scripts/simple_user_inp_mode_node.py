#!/usr/bin/python
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import numpy as np
from pynput.keyboard import Key, Listener
'''
Default values:
'''
y_e_up = 0.06
y_e_down = -0.065
y_e_home = 0
y_e_carry = -0.02
theta_up = 30 * np.pi/180
theta_home = 0
theta_down = -48.01*np.pi/180

key_inp = " "
mode_inp = 1


mode_pub = rospy.Publisher("exec_mode", Float64MultiArray, queue_size=10)
manual_pub = rospy.Publisher("manual_inp", String, queue_size=10)
rospy.init_node("user_mode_pub",anonymous=True)

def on_press(key_press):
    global mode_inp, key_inp, y_e_up, y_e_down, y_e_home, theta_up, theta_home, theta_down
    try:
        key = key_press.char
        if key == "w" or key == "s" or key == "d" or key == "a":
            key_inp = key
        elif key == "1" or key == "2" or key == "3" or key == "4" or key == "5":
            mode_inp = int(key)
            key_inp = " "
        else:
            key_inp = " "
    except:
        print("INVALID INPUT, should be an integer")
        return
            
    if mode_inp == 1: # Down position
        y_e_inp = y_e_down
        theta_inp = theta_home
        task_time_inp = 2
    elif mode_inp == 2: # Excavate
        #mode = 2
        y_e_inp = y_e_carry
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
    mode_param = Float64MultiArray()
    mode_param.data = [mode_inp, y_e_inp, theta_inp, task_time_inp]

    key_inp_str = String()
    key_inp_str.data = key_inp
    mode_pub.publish(mode_param)

    manual_pub.publish(key_inp_str)
    rospy.loginfo(mode_param)
    
def on_release(key):
    key_inp = " "
    key_inp_str = String()
    key_inp_str.data = key_inp
    manual_pub.publish(key_inp_str)

def main():

    while not rospy.is_shutdown():

        # Collect events until released
        with Listener(
                on_press=on_press,
                on_release=on_release) as listener:
            listener.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
