#! /usr/bin/env python
# -*- coding: utf-8 -*-
# limo_application/scripts/lane_detection/lane_detect.py
# WeGo LIMO Pro를 이용한 차선 인식 코드

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist 
from std_msgs.msg import bool

import cv2
import numpy as np

class LaneDetection:
    def __init__(self):
        # ROS Part
        rospy.init_node("mcont_one_lane_detect")
        self.cvbridge = CvBridge()
        self.roi_top = 50
        self.roi_bot = 0
        self.roi_left = 160
        self.roi_right = 480
        self.RED_LANE_LOW_TH = np.array([0,50,50])
        self.RED_LANE_HIGH_TH = np.array([10,255,255])
        rospy.Subscriber(rospy.get_param("~image_topic_name", "/limo/stop_lane_x"), Image, self.image_topic_callback)
        self.distance_pub1 = rospy.Publisher("/topic_name", bool, queue_size=5)
        self.viz = rospy.get_param("~visualization", True)
    

    # return np.ndarray (opencv image type)
    def imageCrop(self, _img=np.ndarray(shape=(480, 640))):
        return _img[self.roi_top:self.roi_bot, self.roi_left:self.roi_right]

    # return np.ndarray (opencv image type)
    def colorDetect(self, _img=np.ndarray(shape=(480, 640)),threshold=0.1):
        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)

        mask_red = cv2.inRange(hls, self.RED_LANE_LOW_TH, self.RED_LANE_HIGH_TH)

        return mask_red
    
    def stop_line_detect(self, _img=np.ndarray(shape=(480,640))):
        
        many_red =  cv2.countNonZero(self.mask_red)
        stop_line = many_red > 5
            
        
    

    def visResult(self):
        cv2.imshow("lane_original", self.frame)
        cv2.imshow("lane_cropped", self.cropped_image)
        cv2.imshow("lane_red", self.mask_red)
        cv2.waitKey(1)

    # ==============================================
    #               Callback Functions
    # ==============================================

    def image_topic_callback(self, img):
        self.frame = self.cvbridge.imgmsg_to_cv2(img, "bgr8")
        self.cropped_image = self.imageCrop(self.frame)
        self.mask_red = self.colorDetect(self.cropped_image)
        self.stop_line_find = self.stop_line_detect(self.mask_red)
        self.distance_pub1.publish((self.stop_line_detectS))

        # visualization
        if self.viz:
            self.visResult()

def run():
    new_class = LaneDetection()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
        rospy.delete_param("roi_top")
        rospy.delete_param("roi_bot")
        rospy.delete_param("roi_left")
        rospy.delete_param("roi_right")
