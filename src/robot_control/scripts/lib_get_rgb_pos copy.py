#!/usr/bin/env python

#   该程序估计不执行main函数，只是单纯的对一些需要的函数进行定义，对图像进行预处理，对所二值化后的轮廓进行一个大小筛选，差不多筛选出兑换站的四个轮廓，再进行一个位置滤波得到四个轮廓
# 以及四个轮廓的中心点

import math
import cv2
import  numpy as np
import itertools
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#获取contours的中心;找出所有轮廓的中心点
def get_center_of_contours(contours):
    sum_x = 0
    sum_y = 0
    num = 0
    for contour in contours:
        for point in contour:
            sum_x+=point[0][0]
            sum_y+=point[0][1]
            num+=1
    center = [sum_x/num,sum_y/num]
    return center


#获取contour中距conter最远点；找出单个轮廓中距离中心点最远的点
def get_farest_point(contour,center):
    farest_length_pow = 0
    for point in contour:
        length_pow = (point[0][0]-center[0])**2 + (point[0][1]-center[1])**2
        if length_pow>farest_length_pow:
            farest_length_pow=length_pow
            point_goal=point[0]
    return point_goal


#获取两点间距离
def len_2point(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


#对contours进行大小滤波；找出所有轮廓中符合面积筛选条件的轮廓
def filt_contours_size(contours,contour_area_min=100,contour_area_max=500):
    filted_small_contours = []
    for contour in contours:
        contour_area = cv2.contourArea(contour)
        if contour_area > contour_area_min and contour_area_max<contour_area_max:
            filted_small_contours.append(contour)
    return filted_small_contours


#对contours进行相对位置滤波；通过取出四个轮廓的中心点，排列出他们所有顺序，查找在哪个顺序连起来只有一个圈，同时进行一个两两中心点的长度比例；返回这四个轮廓和四个中心点
def filt_contours_pos(contours,rate_range = 0.2):  #rate_range: 1+-range
    filted_pos_contours = []
    accord_contours_center_permutation=[]
    #找contour中心
    contour_with_centers = []
    for contour in contours:
        contour_center = [int(x) for x in list(cv2.minAreaRect(contour)[0])]
        contour_with_centers.append([contour, contour_center])
    #对各个中心抽取四个检测
    contours_permutations = [list(x) for x in itertools.permutations(contour_with_centers, 4)]
    for contours_permutation in contours_permutations:
        #按照固定方向，求两点间向量
        vector1=np.array(contours_permutation[0][1])-np.array(contours_permutation[1][1])
        vector2=np.array(contours_permutation[1][1])-np.array(contours_permutation[2][1])
        vector3=np.array(contours_permutation[2][1])-np.array(contours_permutation[3][1])
        vector4=np.array(contours_permutation[3][1])-np.array(contours_permutation[0][1])
        #判断向量连接的圈是一个还是两个，两个则向量叉乘的值变化1与-1，一个则不变化1或-1
        cross_val1=np.cross(vector1,vector2)
        cross_val2=np.cross(vector2,vector3)
        cross_val3=np.cross(vector3,vector4)
        cross_val4=np.cross(vector4,vector1)
        if cross_val1*cross_val2<0 or cross_val2*cross_val3<0 or cross_val3*cross_val4<0 or cross_val4*cross_val1<0:
            continue
        #按照同样固定方向，求两点间距，求两点间向量
        len1 = len_2point(contours_permutation[0][1], contours_permutation[1][1])
        len2 = len_2point(contours_permutation[1][1], contours_permutation[2][1])
        len3 = len_2point(contours_permutation[2][1], contours_permutation[3][1])
        len4 = len_2point(contours_permutation[3][1], contours_permutation[0][1])
        lens = [len1, len2, len3, len4]
        lens.sort() #min->max
        len_max=lens[-1]
        # 判断各段长度与最大长度的比值是否在范围
        flag = 1
        for len in lens:
            rate = len / len_max
            if rate > (1 + rate_range) or rate < (1 - rate_range):
                flag = 0
                break
        if flag == 0:
            continue
        # 得到满足比值的一组排序
        filted_pos_contours = [x[0] for x in contours_permutation]
        accord_contours_center_permutation = [x[1] for x in contours_permutation]
        break
    return filted_pos_contours,accord_contours_center_permutation


# 图像预处理并进行大小、位置筛选，得到四个轮廓以及四个轮廓的中心点，通过查找每个轮廓距离中心点最远的为顶点，进行保存
#获取方框4顶点
#input:
#output:
def get_rgb_pos(img):
    #获取二值化图像
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_min = np.array([132 , 82 , 82])
    hsv_max = np.array([201, 167, 170])
    # hsv_min = np.array([ 43 ,  0 ,230])
    # hsv_max = np.array([ 94 ,130 ,255])
    img_mask = cv2.inRange(img_hsv, hsv_min, hsv_max)
    #形态学滤波
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
    img_close= cv2.morphologyEx(img_mask, cv2.MORPH_CLOSE, kernel)
    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
    # img_close= cv2.morphologyEx(img_mask, cv2.MORPH_CLOSE, kernel)
    img_show = cv2.cvtColor(img_close.copy(), cv2.COLOR_GRAY2BGR)
    # 找到图形边缘,CHAIN_APPROX_SIMPLE取点少
    contours, hierarchy = cv2.findContours(img_close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img_show, contours, -1, (0, 0, 255), 1)
    if len(contours)<4:
        return -1,img_show
    # 大小，位置滤波
    size_filted_contours = filt_contours_size(contours, contour_area_min=80,contour_area_max=500)
    cv2.drawContours(img_show, size_filted_contours, -1, (255, 0, 0), 1)
    if len(size_filted_contours)<4:
        return -2,img_show
    rospy.loginfo("===rgb find contours:%d\n",len(contours))
    rospy.loginfo("===rgb size filted contours:%d\n",len(size_filted_contours))
    pos_filted_contours,centers_permutation = filt_contours_pos(size_filted_contours, rate_range=0.1)
    #
    if len(pos_filted_contours)==0:
        return -3,img_show
    cv2.drawContours(img_show, pos_filted_contours, -1, (0, 255, 0), 1)
    cv2.line(img_show,centers_permutation[0],centers_permutation[1],(0, 255, 0),1)
    cv2.line(img_show,centers_permutation[1],centers_permutation[2],(0, 255, 0),1)
    cv2.line(img_show,centers_permutation[2],centers_permutation[3],(0, 255, 0),1)
    cv2.line(img_show,centers_permutation[3],centers_permutation[0],(0, 255, 0),1)
    # 根据contours找到面积最大时候的顶点
    point_goals=[]
    center = get_center_of_contours(pos_filted_contours)
    for contour in pos_filted_contours:
        point_goal = get_farest_point(contour, center)
        point_goals.append(list(point_goal))
        cv2.drawMarker(img_show, (int(point_goal[0]), int(point_goal[1])), [0, 255, 0], markerType=0, thickness=1)
    cv2.drawMarker(img_show, (int(center[0]), int(center[1])), [0, 255, 0], markerType=0, thickness=1)
    return point_goals,img_show


# 订阅者的串口回调，从话题“/camera/color/image_raw"中读取到的图像进行处理，同时通过话题“rgb_img_show_debug”对处理过的Image进行发布；估计总程序不执行该main函数。
def callback(img):
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(img,'bgr8')
    points,img_show=get_rgb_pos(cv_img)
    try:
        if points<0:
            rospy.logwarn("=================================get rgb pos return err val%d\n",points)
    except Exception as e:
        pass
    ros_img=bridge.cv2_to_imgmsg(img_show)
    pub_img_show.publish(ros_img)

if __name__ == "__main__":
    rospy.init_node('get_rgb_pos', anonymous=True)
    #收听数据：设定收听话题
    Subscriber_rgb=rospy.Subscriber('/camera/color/image_raw', Image, callback) 
    pub_img_show= rospy.Publisher("rgb_img_show_debug",Image, queue_size=10)
    #收听完成后关闭该节点
    rospy.spin()

    