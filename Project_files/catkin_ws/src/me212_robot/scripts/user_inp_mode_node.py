#!/usr/bin/python

import rospy
from std_msgs.msg import Int16

def main():
    mode_pub = rospy.Publisher('ee_mode', Int16, queue_size=10)
    rospy.init_node('user_mode_pub',anonymous=True)

    while not rospy.is_shutdown():
        inp_mode = int(raw_input("Enter desired end effector mode?"))
        mode_pub.publish(inp_mode)
        rospy.loginfo(inp_mode) try:
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    