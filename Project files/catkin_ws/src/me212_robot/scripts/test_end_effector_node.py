# This code makes a node called ee_coord_publisher that publishes a float64multiarray containing [mode, desired_y_e, desired_theta] 

import rospy
from std_msgs.msg import Float64MultiArray

lower_ye = -0.1
upper_ye = 0.2
theta_carry = 48.01
theta_home = 0
theta_dump = -48.01

def main():
    ee_coord_pub = rospy.Publisher('ee_mode', Float64MultiArray, queue_size=10)
    rospy.init_node('ee_coord_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    mode_data = Float64MultiArray()
    while not rospy.is_shutdown():
        mode = 1
        y_e = lower_ye
        theta = theta_home


        #mode = 2
        #y_e = lower_ye
        #theta = theta_carry

        #mode = 3
        #y_e = upper_ye
        #theta = theta_carry

        #mode = 4
        #y_e = upper_ye
        #theta = theta_dump
        
        mode_data.data = [mode, y_e, theta]
        rospy.loginfo(mode_data)
        ee_coord_pub.publish(mode_data)
        rate.sleep()
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass