#!/usr/bin/env python3
import math
import cv2
import  numpy as np
import itertools
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import sqrt, pow, acos

#求两向量夹角
def angle_of_vector(v1, v2):
    pi = 3.1415
    vector_prod = v1[0] * v2[0] + v1[1] * v2[1]
    length_prod = sqrt(pow(v1[0], 2) + pow(v1[1], 2)) * sqrt(pow(v2[0], 2) + pow(v2[1], 2))
    cos = vector_prod * 1.0 / (length_prod * 1.0 + 1e-6)
    return (acos(cos) / pi) * 180

#获取contours的中心；通过轮廓的所有点去寻找中心点
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

#获取contour中距conter最远点
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

#对contours进行大小滤波
def filt_contours_size(contours,contour_area_min=100,contour_area_max=500):
    # 创建一个轮廓列表，如果面积符合筛选条件，则将该轮廓放入列表中
    filted_small_contours = []
    for contour in contours:
        contour_area = cv2.contourArea(contour)
        if contour_area > contour_area_min and contour_area<contour_area_max:
            filted_small_contours.append(contour)
    return filted_small_contours

# 对contours进行相对位置滤波
def filt_contours_pos(contours, rate_range=0.2,ang_range=20):  # rate_range: 1+-range
    # 对各个contours抽取四个检测
    # itertools.permutations(iterable, r) iterable是要生成排列的序列，r是生成排列元素个数
    contours_permutations = [list(x) for x in itertools.permutations(contours, 4)]
    for contours_permutation in contours_permutations:
        top_points_permutation=[]
        #找到四个色块顶点
        # get_center_of_contours是自定义函数
        center=get_center_of_contours(contours_permutation)
        for contour in contours_permutation:
            # 通过找最远的点，从而确定该轮廓的顶点
            top_point=get_farest_point(contour, center)
            top_points_permutation.append(top_point)
        # 按照固定方向，求两点间向量
        vector1 = np.array(top_points_permutation[0]) - np.array(top_points_permutation[1])
        vector2 = np.array(top_points_permutation[1]) - np.array(top_points_permutation[2])
        vector3 = np.array(top_points_permutation[2]) - np.array(top_points_permutation[3])
        vector4 = np.array(top_points_permutation[3]) - np.array(top_points_permutation[0])
        # 判断向量连接的圈是一个还是两个，两个则向量叉乘的值变化1与-1，一个则不变化1或-1
        cross_val1 = np.cross(vector1, vector2)
        cross_val2 = np.cross(vector2, vector3)
        cross_val3 = np.cross(vector3, vector4)
        cross_val4 = np.cross(vector4, vector1)
        if cross_val1 * cross_val2 < 0 or cross_val2 * cross_val3 < 0 or cross_val3 * cross_val4 < 0 or cross_val4 * cross_val1 < 0:
            continue
        #判断向量夹角是否在90左右
        ang1 = angle_of_vector(vector1, vector2)
        ang2 = angle_of_vector(vector2, vector3)
        ang3 = angle_of_vector(vector3, vector4)
        ang4 = angle_of_vector(vector4, vector1)
        #rospy.loginfo([ang1,ang2,ang3,ang4])
        if  abs(ang1)<90-ang_range or abs(ang1)>90+ang_range or \
            abs(ang2)<90-ang_range or abs(ang2)>90+ang_range or \
            abs(ang3)<90-ang_range or abs(ang3)>90+ang_range or \
            abs(ang4)<90-ang_range or abs(ang4)>90+ang_range:
            continue
        # 按照同样固定方向，求两点间距，求两点间向量
        len1 = len_2point(top_points_permutation[0], top_points_permutation[1])
        len2 = len_2point(top_points_permutation[1], top_points_permutation[2])
        len3 = len_2point(top_points_permutation[2], top_points_permutation[3])
        len4 = len_2point(top_points_permutation[3], top_points_permutation[0])
        lens = [len1, len2, len3, len4]
        # rospy.loginfo(lens)
        lens_average=(len1+len2+len3+len4)/4
        # 判断各段长度与最大长度的比值是否在范围
        flag = 1
        for len in lens:
            rate = len / lens_average
            #rospy.loginfo("+++++++++++++++++++++++++++++++++length rate:%f\n",rate)
            if rate > (1 + rate_range) or rate < (1 - rate_range):
                flag = 0
                break
        if flag == 0:
            continue
        # 得到满足比值的一组排序
        return contours_permutation, top_points_permutation
    return [],[]

# 模板匹配，滤除离群点,找到方框包围matches
def sift_match_filt_outliers(img,template,filt_coefficient=1.5,rect_expand_coefficient=0.1):
    # sift关键点检测 #kp关键点信息包括方向，尺度，位置信息 #des是关键点的描述符
    sift = cv2.SIFT_create()
    kp_template, des_template_gray = sift.detectAndCompute(template, None)
    kp_img, des_img_gray = sift.detectAndCompute(img, None)
    # 匹配 #match中保存距离(小好)，原点，目标点
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)  # 交叉最佳匹配
    matches = bf.match(des_template_gray, des_img_gray)  # 匹配描述符
    #平均点
    xsum=0
    ysum=0
    for match in matches:
        train_index=match.trainIdx
        train_pt = kp_img[train_index].pt
        xsum += train_pt[0]
        ysum += train_pt[1]
    center=[xsum/len(matches),ysum/len(matches)]
    #各点到平均点的偏差；求偏差平均
    distances=[]
    distances_sum=0
    for match in matches:
        train_index=match.trainIdx
        train_pt = kp_img[train_index].pt #xy
        distance = len_2point(train_pt, center)
        distances.append(distance)
        distances_sum+=distance
    distances_average=distances_sum/len(matches)
    #大于平均偏差filt_coefficient倍的为离群，去除
    result=[]
    for distance,match in zip(distances,matches):
        if distance<distances_average*filt_coefficient:
            result.append(match)
    #显示result
    # img = cv2.drawMatches(template, kp_template,img, kp_img,  result, None,flags=2)
    # cv2.imshow(" ",img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
    #找到最大的方框包围result
    up = kp_img[result[0].trainIdx].pt[1]
    down = kp_img[result[0].trainIdx].pt[1]
    left = kp_img[result[0].trainIdx].pt[0]
    right = kp_img[result[0].trainIdx].pt[0]
    for match in result:
        index = match.trainIdx
        train_pt = kp_img[index].pt #xy
        if train_pt[1]<up:
            up=train_pt[1]
        if train_pt[1]>down:
            down = train_pt[1]
        if train_pt[0]<left:
            left=train_pt[0]
        if train_pt[0]>right:
            right=train_pt[0]
    #扩大框的大小
    up = int(up - (down - up) * rect_expand_coefficient)
    if up<0:
        up=0
    left = int(left - (right - left) * rect_expand_coefficient)
    if left<0:
        left=0
    down = int(down + (down - up) * rect_expand_coefficient)
    if down > 479:
        down = 479
    right = int(right + (right - left) * rect_expand_coefficient)
    if right > 639:
        right = 639
    return [up,left],[down,right]

#获取方框4顶点
#input:
#output:
def get_rgb_pos(img,up_left,down_right):
    #获取二值化图像
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # hsv_min = np.array([132 , 82 , 82])
    # hsv_max = np.array([201, 167, 170])
    hsv_min = np.array([ 43 ,  0 ,230])
    hsv_max = np.array([ 94 ,130 ,255])
    img_mask = cv2.inRange(img_hsv, hsv_min, hsv_max)
    #形态学滤波
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
    img_close= cv2.morphologyEx(img_mask, cv2.MORPH_CLOSE, kernel)
    img_show = cv2.cvtColor(img_close.copy(), cv2.COLOR_GRAY2BGR)
    # 标记方框，除去方框外的点
    cv2.rectangle(img_show,[up_left[1],up_left[0]],[down_right[1],down_right[0]],[0,255,0],2)
    img_close[:,:up_left[1]] = 0
    img_close[:,down_right[1]:] = 0
    img_close[:up_left[0],:] = 0
    img_close[down_right[0]:,:] = 0
    # 找到图形边缘,CHAIN_APPROX_SIMPLE取点少
    contours, hierarchy = cv2.findContours(img_close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img_show, contours, -1, (0, 0, 255), 1)
    if len(contours)<4:
        return -1,img_show
    # 大小滤波
    size_filted_contours = filt_contours_size(contours, contour_area_min=150,contour_area_max=10000)
    cv2.drawContours(img_show, size_filted_contours, -1, (255, 0, 0), 1)
    if len(size_filted_contours)<4:
        return -2,img_show
    rospy.loginfo("+++++++++++++++++++++++++++++++++rgb size filted contours:%d\n",len(size_filted_contours))
    #位置滤波  
    pos_filted_contours,centers_permutation = filt_contours_pos(size_filted_contours, rate_range=0.25,ang_range=10)
    if len(pos_filted_contours)==0:
        return -3,img_show
    cv2.drawContours(img_show, pos_filted_contours, -1, (0, 255, 0), 1)
    cv2.line(img_show,centers_permutation[0],centers_permutation[1],(0, 255, 0),1)
    cv2.line(img_show,centers_permutation[1],centers_permutation[2],(0, 255, 0),1)
    cv2.line(img_show,centers_permutation[2],centers_permutation[3],(0, 255, 0),1)
    cv2.line(img_show,centers_permutation[3],centers_permutation[0],(0, 255, 0),1)
    return centers_permutation,img_show

def callback(img):
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(img,'bgr8')
    #template = cv2.imread("./src/robot_control/box.jpg")3
    template = cv2.imread("/home/wjx/ros/sw_test/src/robot_control/img_box.jpg")
    up_left,down_right=sift_match_filt_outliers(cv_img.copy(), template.copy(),filt_coefficient=1.5,rect_expand_coefficient=0.1)
    points,img_show=get_rgb_pos(cv_img,up_left,down_right)
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
    Subscriber_rgb=rospy.Subscriber('/camera/color/image_raw', Image, callback,queue_size=1) 
    pub_img_show= rospy.Publisher("rgb_img_show_debug",Image, queue_size=1)
    #收听完成后关闭该节点
    rospy.spin()

    