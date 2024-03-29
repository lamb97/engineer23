#!/usr/bin/env python3
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped
import numpy as np
if __name__ == '__main__':
    rospy.init_node('get_target_world_pose_step1_box_temp2', anonymous=True)
    #等待realsense初始化完成
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    while(not rospy.is_shutdown()):
        try:
            trans=tfBuffer.lookup_transform("camera_color_optical_frame","box_temp",rospy.Time())
            break
        except Exception as e:
            pass
    rospy.loginfo("=================================wait for box_temp broadcasting ends \n")
    #发布box_temp 到 box_temp2
    broadcaster_box_temp2 = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "box_temp"
    static_transformStamped.child_frame_id = "box_temp2"
    static_transformStamped.transform.translation.x = 0
    static_transformStamped.transform.translation.y = 0
    static_transformStamped.transform.translation.z = 0
    R_box_temp2=Rotation.from_euler("zyx",[0,0,180],True) #根据box坐标系设置是否180
    Q_box_temp2=Rotation.as_quat(R_box_temp2)
    static_transformStamped.transform.rotation.x = Q_box_temp2[0]
    static_transformStamped.transform.rotation.y = Q_box_temp2[1]
    static_transformStamped.transform.rotation.z = Q_box_temp2[2]
    static_transformStamped.transform.rotation.w = Q_box_temp2[3]
    broadcaster_box_temp2.sendTransform(static_transformStamped)
    rospy.spin()
 