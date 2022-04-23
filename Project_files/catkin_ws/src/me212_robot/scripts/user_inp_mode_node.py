#!/usr/bin/python

import rospy
from std_msgs.msg import Int16

def main():
    mode_pub = rospy.Publisher('exec_mode', Int16, queue_size=10)
    rospy.init_node('user_mode_pub',anonymous=True)

    while not rospy.is_shutdown():
        try:
            inp_mode = int(raw_input("Enter desired end effector mode: "))
        except ValueError:
            print("INVALID INPUT, should be an integer")
            continue
            
        if inp_mode < 1 or inp_mode > 4:
            print("INVALID INPUT, should be an integer between 1 and 4")
            continue
        mode_pub.publish(inp_mode)
        rospy.loginfo(inp_mode)
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
