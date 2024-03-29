#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
import cv2
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from robot_control.srv import *
from robot_control.msg import target_position_pose
from geometry_msgs.msg import PointStamped,TransformStamped,PoseStamped
from scipy.spatial.transform import Rotation
from lib_get_rgb_pos import get_rgb_pos,sift_match_filt_outliers
from lib_get_basketframe_pos import get_basketframe_pos
from lib_get_pointcloud_pose import rigid_transform_3D


# 回调函数
def callback_synchronize(Subscriber_rgb_data,Subscriber_pointcloud_data):
    #图像转换
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(Subscriber_rgb_data,'bgr8')
    #调用lib获取方框大致位置
    # template = cv2.imread("/home/wjx/ros/sw_test/src/robot_control/box.jpg")
    template = cv2.imread("/home/wjx/ros/sw_test/src/robot_control/img_box.jpg")
    up_left=[0,0]
    down_right=[479,639]
    try:
        up_left,down_right=sift_match_filt_outliers(cv_img.copy(), template.copy(),filt_coefficient=1.5,rect_expand_coefficient=0.1)
    except Exception as e:
        pass
    #调用lib获取rgb四点2d坐标
    points,img_show=get_rgb_pos(cv_img,up_left,down_right)
    ros_img=bridge.cv2_to_imgmsg(img_show)
    pub_img_show.publish(ros_img)
    try:
        if points<0:
            rospy.logwarn("=================================get rgb pos return err val%d\n",points)
            return
    except Exception as e:
        pass
    #服务器提供点云四点3d坐标   #!!!rgb,点云参考系在camera_color_optical_frame
    point1_pos = client(Subscriber_pointcloud_data,points[0][0],points[0][1])# 在相机坐标系下的三维坐标
    point2_pos = client(Subscriber_pointcloud_data,points[1][0],points[1][1])
    point3_pos = client(Subscriber_pointcloud_data,points[2][0],points[2][1])
    point4_pos = client(Subscriber_pointcloud_data,points[3][0],points[3][1])
    point1_pos = [point1_pos.x,point1_pos.y,point1_pos.z]
    point2_pos = [point2_pos.x,point2_pos.y,point2_pos.z]
    point3_pos = [point3_pos.x,point3_pos.y,point3_pos.z]
    point4_pos = [point4_pos.x,point4_pos.y,point4_pos.z]
    #求方框坐标系下顶点坐标new_3d
    points_3d=[point1_pos,point2_pos,point3_pos,point4_pos]
    new_points_3d,new_origin=get_basketframe_pos(points_3d)
    #求方框位姿3d_pose
    Q=rigid_transform_3D(new_points_3d, points_3d)  #方框位姿
    center=[(point1_pos[0]+point2_pos[0]+point3_pos[0]+point4_pos[0])/4, # 在相机坐标系下框的中心点的坐标
            (point1_pos[1]+point2_pos[1]+point3_pos[1]+point4_pos[1])/4,
            (point1_pos[2]+point2_pos[2]+point3_pos[2]+point4_pos[2])/4,]
    #发布目标在camera_color_optical_frame下的坐标系关系
    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "camera_color_optical_frame"
    static_transformStamped.child_frame_id = "box_temp"
        
    static_transformStamped.transform.translation.x = center[0] # 将相机坐标系原点移动到框的中心点
    static_transformStamped.transform.translation.y = center[1]
    static_transformStamped.transform.translation.z = center[2]
    static_transformStamped.transform.rotation.x = Q[0]         # 进行一个姿态变换，将移动到的方框坐标系旋转至与原方框坐标系一样的方向
    static_transformStamped.transform.rotation.y = Q[1]
    static_transformStamped.transform.rotation.z = Q[2]
    static_transformStamped.transform.rotation.w = Q[3]
    broadcaster.sendTransform(static_transformStamped)          # 进行广播器发布数据
    #等待，直到ros shutdown，保证box静态tf存在
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        #发布识别图片，静态
        pub_img_show.publish(ros_img)
        #发布识别四顶点，静态
        pointstamped0=PointStamped()
        pointstamped1=PointStamped()
        pointstamped2=PointStamped()
        pointstamped3=PointStamped()
        points_3d=[point1_pos,point2_pos,point3_pos,point4_pos]
        pointstampeds=[pointstamped0,pointstamped1,pointstamped2,pointstamped3]
        pub=[pub0,pub1,pub2,pub3]
        for apoint,pointstamped,apub in zip(points_3d,pointstampeds,pub):
            pointstamped.header.stamp=rospy.Time.now()
            pointstamped.header.frame_id="camera_color_optical_frame"
            pointstamped.point.x=apoint[0] 
            pointstamped.point.y=apoint[1]
            pointstamped.point.z=apoint[2]
            apub.publish(pointstamped)

 
if __name__ == '__main__':
    rospy.init_node('get_target_pose', anonymous=True)
    #等待realsense初始化完成
    rospy.wait_for_message('/camera/color/image_raw',Image,timeout=None) # “/camera/color/image_raw”为深度相机的话题
    rospy.sleep(4)
    rospy.loginfo("=================================wait for realsense topic ends \n")
    #请求服务：设定请求服务类型，服务参数
    client = rospy.ServiceProxy("get_pointcloud_3d_service",get_pointcloud_3d) # 设定客户端，打算从“get_pointcloud_3d_service”中得到3d信息
    #收听数据：设定收听话题
    Subscriber_rgb=message_filters.Subscriber('/camera/color/image_raw', Image, queue_size=1) 
    Subscriber_pointcloud=message_filters.Subscriber('/camera/depth/color/points', PointCloud2, queue_size=1) 
    Synchronizer = message_filters.ApproximateTimeSynchronizer([Subscriber_rgb, Subscriber_pointcloud],1,0.1,allow_headerless=True)
    Synchronizer.registerCallback(callback_synchronize)#ros master处注册两个话题的同一个回调
    #发布camera_color_optical_frame到box_temp的坐标变化
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    pub_img_show= rospy.Publisher("rgb_img_show",Image, queue_size=10)
    #发布四顶点
    pub0 = rospy.Publisher("top_point0",PointStamped, queue_size=10)
    pub1 = rospy.Publisher("top_point1",PointStamped, queue_size=10)
    pub2 = rospy.Publisher("top_point2",PointStamped, queue_size=10)
    pub3 = rospy.Publisher("top_point3",PointStamped, queue_size=10)
    
    rospy.spin()
 