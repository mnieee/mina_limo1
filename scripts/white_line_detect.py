#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2 #OpenCV 모듈을 불러오는 것


class White_line_Detect:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("red_line_node")
        self.pub = rospy.Publisher("/red/compressed", CompressedImage, queue_size=10)
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.img_CB)

    def detect_color(self, img):
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # 직접 구현한 BGR2HSVd와 OpenCV의 cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define range of white color in HSV
        red_lower = np.array([9, 68, 92]) #hsv 순서로
        red_upper = np.array([6, 82, 58]) 

        # Threshold(한계점) the HSV image to get only white colors
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        cv2.imshow("img", img)
        cv2.imshow("red_mask", red_mask)
        red_color = cv2.bitwise_and(img, img, mask=red_mask)
        cv2.imshow("red_color", red_color)
        return red_color

    def img_warp(self, img):
        self.img_x, self.img_y = img.shape[1], img.shape[0]
        # print(f'self.img_x:{self.img_x}, self.img_y:{self.img_y}')

        img_size = [640, 480]
        # ROI(관심영역)
        src_side_offset = [0, 240]
        src_center_offset = [200, 315]
        src = np.float32(
            [
                [0, 479],
                [src_center_offset[0], src_center_offset[1]],
                [640 - src_center_offset[0], src_center_offset[1]],
                [639, 479],
            ]
        )
        # 아래 2 개 점 기준으로 dst 영역을 설정합니다.
        dst_offset = [round(self.img_x * 0.125), 0]
        # offset x 값이 작아질 수록 dst box width 증가합니다.
        dst = np.float32(
            [
                [dst_offset[0], self.img_y],
                [dst_offset[0], 0],
                [self.img_x - dst_offset[0], 0],
                [self.img_x - dst_offset[0], self.img_y],
            ]
        )
        # find perspective matrix
        matrix = cv2.getPerspectiveTransform(src, dst)
        matrix_inv = cv2.getPerspectiveTransform(dst, src)
        warp_img = cv2.warpPerspective(img, matrix, [self.img_x, self.img_y])
        return warp_img

    def img_CB(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data)
        warp_img = self.img_warp(img)
        white_color = self.detect_color(warp_img)
        white_line_img_msg = self.bridge.cv2_to_compressed_imgmsg(white_color)
        self.pub.publish(white_line_img_msg)
        # cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.namedWindow("white_color", cv2.WINDOW_NORMAL)
        # cv2.imshow("img", img)
        cv2.imshow("white_color", white_color)
        cv2.waitKey(1)


if __name__ == "__main__":
    white_line_detect = White_line_Detect()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass