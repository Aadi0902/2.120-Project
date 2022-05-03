#!/usr/bin/python
import rospy
import cv2
import numpy as np

# conversion from ros image message to opencv image
# -------------
# from cv_bridge import CvBridge
# bridge = CvBridge()
# cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
# we might need to change 'passthrough'
# see : http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# -------------

def regolith_detector(cv_image, h_low = 10, h_high = 90, fraction_threshold = 0.10, window_xc = 960, window_yc =1000, window_w = 480, window_h = 10):
    is_filled = False
    window = cv_image[window_yc-window_h:window_yc+window_h, window_xc-window_w:window_xc+window_w,:]
    window_hsv = cv2.cvtColor(window, cv2.COLOR_BGR2HSV)
    index = np.logical_and(window_hsv[:,:,0]>h_low, window_hsv[:,:,0]<h_high)
    is_filled = index.sum() >= fraction_threshold*2*window_w*2*window_h
    return is_filled
