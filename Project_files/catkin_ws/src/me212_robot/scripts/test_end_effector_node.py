#!/usr/bin/python
# This code makes a node called ee_coord_publisher that publishes a float64multiarray containing [mode, desired_y_e, desired_theta] 

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool


pos_status = False

#def callback(pos_reached):
    #pos_status = pos_reached;
    #rospy.loginfo(pos_status)
    
def main():
    pi = 22/7;
    lower_ye = -0.07
    upper_ye = 0.07
    theta_carry = 48.01 * pi/180;
    theta_home = 0
    theta_dump = -48.01*pi/180;
    ee_coord_pub = rospy.Publisher('ee_mode', Float64MultiArray, queue_size=10)
    #rospy.Subscriber("Position_reached", Bool, callback)
    
    rospy.init_node('ee_coord_publisher', anonymous=True)
    rate = rospy.Rate(0.15) #0.12hz # 10hz

    mode_data = Float64MultiArray()
    mode = 2
    while not rospy.is_shutdown():
        
        #if pos_status == True:
           # mode = mode + 1
            
        
        if mode == 1: # Home position
            y_e = lower_ye
            theta = theta_home
            task_time = 0.5
        elif mode == 2: # Excavate
            #mode = 2
            y_e = lower_ye
            theta = theta_carry
            task_time = 0.5
        elif mode == 3: # Straigh line up
            #mode = 3
            y_e = upper_ye
            theta = theta_carry
            task_time = 4
        elif mode ==4: # Dump
            #mode = 4
            y_e = upper_ye
            theta = theta_dump
            task_time = 0.5
        
        mode_data.data = [mode, y_e, theta, task_time]
        if mode == 4:
            mode = 0
        mode = mode + 1
        rospy.loginfo(mode_data)
        ee_coord_pub.publish(mode_data)
        rate.sleep()
        
        #if pos_status == True:
            #rospy.sleep(0.3) # 0.3 seconds
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
