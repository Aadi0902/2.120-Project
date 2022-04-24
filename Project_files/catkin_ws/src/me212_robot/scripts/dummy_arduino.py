#!/usr/bin/python
import rospy
from std_msgs.msg import Float64MultiArray

def main():
    dummy_pub = rospy.Publisher("encoder_current_val", Float64MultiArray, queue_size=10)
    rospy.init_node("dummy_arduino",anonymous=True)
    dummy_val = Float64MultiArray()
    rate = rospy.Rate(20) # 20 Hz
    while not rospy.is_shutdown():
        dummy_val.data = [0.0 0.0 0.0 0.0] # [enc1 enc2 cur1 cur2]
        rospy.loginfo(dummy_val)
        dummy_pub.publish(dummy_val)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
