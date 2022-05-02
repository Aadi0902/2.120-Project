#!/usr/bin/python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D

pose_pub = rospy.Publisher("robot_pose2d", Pose2D, queue_size=10)

rospy.init_node("set_point_pub_sub", anonymous=True)


def pose_callback(pose_val):
    pose_val_2d = Pose2D()
    pose_val_2d.x = pose_val.pose.position.x
    pose_val_2d.y = pose_val.pose.position.y
    
    x = pose_val.pose.orientation.x
    y = pose_val.pose.orientation.y
    z = pose_val.pose.orientation.z
    w = pose_val.pose.orientation.w
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    pose_val_2d.theta = yaw_z
    pose_pub.publish(pose_val_2d)
    rospy.loginfo(pose_val_2d)

def main():
    rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
