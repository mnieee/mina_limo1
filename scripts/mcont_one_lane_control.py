#!/usr/bin/env python
# -*- coding:utf-8 -*-
# limo_application/scripts/lane_detection/control.py
# WeGo LIMO Pro를 이용한 주행 코드

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist 
from std_msgs.msg import bool

import cv2
import numpy as np


import math

class LimoController:
    def __init__(self):
        rospy.init_node("stop_line_detect")
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
        self.BASE_SPEED = rospy.get_param('base_speed', 0.1)
        rospy.Subscriber("/limo/lane?", Int32, self.lane_callback)
        rospy.Subscriber("/limo/stop?", Int32, self.stop_topic_callback)
        self.drive_pub = rospy.Publisher(rospy.get_param("~control_topic_name", "/cmd_vel?"), Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.03), self.drive_callback)
    # ==============================================
    #               Callback Functions
    # ==============================================

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

        return stop_line


    def drive_callback(self, _event):
        '''
            입력된 데이터를 종합하여,
            속도 및 조향을 조절하여 최종 cmd_vel에 Publish
        '''

        drive_data = Twist()
        drive_data.angular.z = self.distance_to_ref * self.LATERAL_GAIN * 0.001

        try:
            if self.stop_line_detect:
                drive_data.linear.x = 0.0
                drive_data.angular.z = 0.0
                rospy.logwarn("Stop line Detected, Stop!")
            else:
                drive_data.linear.x = self.BASE_SPEED
                print("Auto Driving! :", self.BASE_SPEED)


        except Exception as e:
            rospy.logwarn(e)

def run():
    new_class = LimoController()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
        rospy.delete_param('base_speed')
