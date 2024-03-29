#!/usr/bin/env python3
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation
from tf2_geometry_msgs import PoseStamped as tf2_PoseStamped
from tf2_geometry_msgs import Vector3Stamped as tf2_Vector3Stamped
from robot_control.msg import move_pose_steps
from geometry_msgs.msg import PoseStamped,Vector3Stamped,TransformStamped
import numpy as np
if __name__ == '__main__':
    rospy.init_node('get_target_world_pose', anonymous=True)
    #box坐标系到world的tf
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    #box位姿在world中
    pub=rospy.Publisher("box_pose_in_world",move_pose_steps,queue_size=10)
    rate=rospy.Rate(10)
    #box坐标系到world的tf
    while(not rospy.is_shutdown()):# 使兑换框的坐标系的z轴朝向你车的方向
        try:
            #获取box_temp姿态是否正确！！！要求boxz轴要与worldx轴点积为正；要根据boxtemp的姿态，设置box
            vector_world=tf2_Vector3Stamped()
            vector_world.header.frame_id="world"
            vector_world.header.stamp=rospy.Time()
            vector_world.vector.x=1
            vector_world.vector.y=0
            vector_world.vector.z=0
            vector_world_x=tfBuffer.transform(vector_world,"box_temp")# 该坐标点从世界坐标系到兑换框坐标系的新坐标
            dot_vector_world_x_vector_box_temp_z= np.dot([0,0,1],[vector_world_x.vector.x,vector_world_x.vector.y,vector_world_x.vector.z])           
            #发布box姿态，如果小于90度
            if dot_vector_world_x_vector_box_temp_z>0:
                while(not rospy.is_shutdown()):
                    try:
                        trans=tfBuffer.lookup_transform("world","box_temp",rospy.Time())
                        trans.header.stamp=rospy.Time.now()
                        trans.child_frame_id="box"
                        broadcaster.sendTransform(trans)
                        rospy.loginfo("=================================choosing box_temp or box_temp2 ends : box_temp \n")
                        break
                    except Exception as e:
                        # rospy.loginfo(e) 
                        rate.sleep() 
            #发布box姿态，如果大于90
            else:
                while(not rospy.is_shutdown()):
                    try:
                        trans=tfBuffer.lookup_transform("world","box_temp2",rospy.Time())
                        trans.header.stamp=rospy.Time.now()
                        trans.child_frame_id="box"
                        broadcaster.sendTransform(trans)
                        rospy.loginfo("=================================choosing box_temp or box_temp2 ends : box_temp2 \n")
                        break
                    except Exception as e:
                        # rospy.loginfo(e) 
                        rate.sleep()    
            break
        except Exception as e:
            # rospy.loginfo(e)    
            rate.sleep()
    #box位姿在world中
    published_time=0
    while(not rospy.is_shutdown()):
        try:
            amove_pose_steps=move_pose_steps()
            #box系下的目标pose  !!!另z轴方向调换，另末端坐标系靠近box系时，末端姿态正确
            posestamped=PoseStamped()
            posestamped.header.stamp=rospy.Time.now()
            posestamped.header.frame_id="box"
            posestamped.pose.position.x=0
            posestamped.pose.position.y=0
            posestamped.pose.position.z=0
            R=Rotation.from_euler("zyx",[0,0,180],True)
            Q=Rotation.as_quat(R)
            posestamped.pose.orientation.x=Q[0]
            posestamped.pose.orientation.y=Q[1]
            posestamped.pose.orientation.z=Q[2]
            posestamped.pose.orientation.w=Q[3]
            #世界坐标系下的目标pose
            posestamped_move_step2=tfBuffer.transform(posestamped,"world") #最终的目标姿态，完全塞进
            posestamped.pose.position.z=-0.2
            posestamped_move_step1=tfBuffer.transform(posestamped,"world") #没塞进去的姿态
            amove_pose_steps.step1=posestamped_move_step1
            amove_pose_steps.step2=posestamped_move_step2
            pub.publish(amove_pose_steps)
            if published_time==0:
                rospy.loginfo("=================================publishing box pose topic ok , not ends \n")
                published_time=1
        except Exception as e:
            # rospy.loginfo(e)    
            rate.sleep()  
    rospy.spin()
 