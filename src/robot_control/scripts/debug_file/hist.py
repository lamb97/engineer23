#!/usr/bin/env python
import cv2,rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback(object):
    pass


# def click(event, x, y, flags, para):
#    if events == cv2.EVENT_LBUTTONDOWN:
#        return 1
#    else:
#        return 0


def Choose_Color(cv_img):
    image0 = cv_img
    img = cv2.cvtColor(image0, cv2.COLOR_BGR2HSV)

    img = cv2.resize(img, (int(img.shape[1] / 2), int(img.shape[0] / 2)))

    bgr_img_show=cv2.resize(image0, (int(img.shape[1] / 2), int(img.shape[0] / 2)))
    cv2.imshow("image", bgr_img_show)


    cv2.createTrackbar("H_min", "image", 0, 255, callback)
    cv2.createTrackbar("H_max", "image", 255, 255, callback)

    cv2.createTrackbar("S_min", "image", 0, 255, callback)
    cv2.createTrackbar("S_max", "image", 255, 255, callback)

    cv2.createTrackbar("V_min", "image", 0, 255, callback)
    cv2.createTrackbar("V_max", "image", 255, 255, callback)

    while (True):

        H_min = cv2.getTrackbarPos("H_min", "image", )
        S_min = cv2.getTrackbarPos("S_min", "image", )
        V_min = cv2.getTrackbarPos("V_min", "image", )

        H_max = cv2.getTrackbarPos("H_max", "image", )
        S_max = cv2.getTrackbarPos("S_max", "image", )
        V_max = cv2.getTrackbarPos("V_max", "image", )

        lower_hsv = np.array([H_min, S_min, V_min])
        upper_hsv = np.array([H_max, S_max, V_max])

        mask = cv2.inRange(img, lower_hsv, upper_hsv)

        # print("H_min = %d,H_max = %d,S_min = %d,S_max = %d,V_min = %d,V_max = %d"%(H_min,H_max,S_min,S_max,V_min,V_max))

        cv2.imshow("mask", mask)

        t=cv2.waitKey(1) & 0XFF
        if t == 27 or t == 32:#esc的ASCII码
            cv2.destroyWindow("mask")
            cv2.destroyWindow("image")
            hsv_min = np.array([H_min, S_min, V_min])
            hsv_max = np.array([H_max, S_max, V_max])
            print(hsv_min,hsv_max)
            return hsv_min,hsv_max

def callback(img):
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(img,'bgr8')
    hsv_min,hsv_max=Choose_Color(cv_img)

if __name__ == "__main__":
    rospy.init_node('hist', anonymous=True)
    rospy.sleep(7)
    #收听数据：设定收听话题
    Subscriber_rgb=rospy.Subscriber('/camera/color/image_raw', Image, callback) 
    rospy.spin()

