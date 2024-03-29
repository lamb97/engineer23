#!/usr/bin/env python3
import rospy
import serial
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
from math import pi, tau, dist, fabs, cos
from robot_control.msg import move_pose_steps
import tf2_ros
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import struct
import copy

class robot_moveit_plan:
    def __init__(self,group_name,eef_link_name):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()# 初始化场景对象
        self.move_group = moveit_commander.MoveGroupCommander(group_name)# 初始化需要使用move group控制的robot group 
        self.move_group.set_end_effector_link(eef_link_name)# 设置末端坐标
        self.move_group.set_max_velocity_scaling_factor(1.0)# 设置最大速度缩放因子
        self.move_group.set_max_acceleration_scaling_factor(1.0)# 设置最大加速度缩放因子
        self.move_group.set_goal_position_tolerance(0.01)# 设置目标位置允许误差
        self.move_group.set_goal_orientation_tolerance(0.01)# 设置目标姿态允许误差
        self.move_group.allow_replanning(True)# 是否允许重新规划
        self.move_group.set_planning_time(5)# 设置规划时间
        self.move_group.set_num_planning_attempts(4) #设置并行求解个数最大4个

    def print_config(self):
        planning_frame = self.move_group.get_planning_frame()# 获取执行规划框架的名称
        print("============ Planning frame: %s" % planning_frame)
        eef_link = self.move_group.get_end_effector_link()# 获取末端坐标的名称
        print("============ End effector link: %s" % eef_link)
        group_names = self.robot.get_group_names()# 获取所定义的机器人的组的名称
        print("============ Available Planning Groups:", group_names)

    def create_scene(self,box_length,tool_length):
        self.scene.clear()# 去除场景里的其他障碍物

        tool_size = [tool_length, tool_length, tool_length] # 矿石三维尺寸
        tool_pose = PoseStamped()
        tool_pose.pose.position.x = 0.0                     # 矿石的位置信息
        tool_pose.pose.position.y = 0.0
        tool_pose.pose.position.z = -tool_length / 2
        tool_pose.pose.orientation.x = 1e-6                 # 矿石的位置信息允许误差
        tool_pose.pose.orientation.y = 1e-6
        tool_pose.pose.orientation.z = 1e-6
        tool_pose.pose.orientation.w = 1.000000
        tool_pose.header.frame_id = self.move_group.get_end_effector_link()
        self.scene.attach_box(self.move_group.get_end_effector_link(), 'gold', tool_pose, tool_size)

        #框
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'box'
        box_pose.pose.position.z = box_length/2
        box_pose.pose.orientation.x = 1e-6
        box_pose.pose.orientation.y = get_plan6
        box_pose.pose.position.y = -box_length*0.5
        self.scene.add_box('box2', box_pose, box_size)

        box_size = [0.001, box_length, box_length]
        box_pose.pose.position.x = box_length*0.5
        box_pose.pose.position.y = 1e-6
        self.scene.add_box('box3', box_pose, box_size)
        box_pose.pose.position.x = -box_length*0.5
        box_pose.pose.position.y = 1e-6
        self.scene.add_box('box4', box_pose, box_size)

        box_size = [box_length, box_length, 0.001]
        box_pose.pose.position.x = 1e-6
        box_pose.pose.position.y = 1e-6
        box_pose.pose.position.z = box_length
        self.scene.add_box('box5', box_pose, box_size)

    #末端从当前位置，到达pose
    def get_plan(self,pose_goal):
        #当前位置为起始
        self.move_group.set_start_state_to_current_state() 
        #设置目标位置
        self.move_group.set_pose_target(pose_goal)
        _,path_plan,_,error_code = self.move_group.plan()
        if error_code.val==1:
            rospy.loginfo("=================================path plan ends\n")
            return path_plan
        else:
            rospy.loginfo("=================================path plan error code val %d",error_code.val)
            return None
    #
    def get_cartesian_plan(self,pose_goal):
        wpose = self.move_group.get_current_pose().pose
        wpose.position=pose_goal.pose.position
        plan, fraction = self.move_group.compute_cartesian_path([wpose], 0.01, 0.0)
        rospy.loginfo("=================================path plan ends\n")
        return plan


    def callback(self,amove_pose_steps):
        #第一步规划
        ajointstate=JointState()
        ajointstate.name = ['car_fb2car_yaw', 'car_lr2car_fb','fb_move2roll_link','origin_global2car_lr','pitch_link2yaw_link','roll_link2pitch_link','ud_fixed2ud_move','ud_move2fb_move']
        ajointstate.position = [0,0,0,0,0,3.14*90/180,0.15,0]
        pub_joint_state.publish(ajointstate)
        rospy.sleep(1)
        while(not rospy.is_shutdown()):
            plan_step1 = self.get_plan(amove_pose_steps.step1)
            if plan_step1:
                break
        #第二步规划
        ajointstate.name=plan_step1.joint_trajectory.joint_names
        ajointstate.position=plan_step1.joint_trajectory.points[-1].positions
        pub_joint_state.publish(ajointstate)
        rospy.sleep(1)
        while(not rospy.is_shutdown()):
            plan_step2 = self.get_cartesian_plan(amove_pose_steps.step2)
            if plan_step2:
                break
        #发布完整规划信息，发送所有点-每个点有各个关节的位置
        length=len(plan_step1.joint_trajectory.points)+len(plan_step2.joint_trajectory.points)
        for index,point in enumerate(plan_step1.joint_trajectory.points+plan_step2.joint_trajectory.points):
            #发送一点
            for ajoint_pos in list(point.positions):
                four_bytes=struct.pack('f',ajoint_pos) #float转为4bytes
                ser.write(four_bytes)
            if index+1==length:
                dd=struct.pack('B',0xdd)
                aa=struct.pack('B',0xaa)
                ser.write(dd)
                ser.write(aa)
            else:
                d=struct.pack('B',0x0d)
                a=struct.pack('B',0x0a)
                ser.write(d)
                ser.write(a)
        while(not rospy.is_shutdown()):
            pass

if __name__ == "__main__":
    rospy.init_node("get_robot_path", anonymous=True)
    #初始化规划器
    robot=robot_moveit_plan("robot","hand")
    robot.print_config()
    #等待box坐标系
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    while(not rospy.is_shutdown()):
        try:
            trans=tfBuffer.lookup_transform("world","box",rospy.Time())# 从box到world的转换
            rospy.loginfo("=================================wait for box broadcasting,ends \n")
            break
        except Exception as e:
            pass
    #开启串口
    rate=rospy.Rate(0.5)
    while(not rospy.is_shutdown()):
        try:
            ser=serial.Serial("/dev/ttyUSB2",115200,timeout=1000) #默认一字节一次
            rospy.loginfo("=============================Serial Port initialization, ends \n")
            break
        except Exception as e:
            rospy.logerr("=============================Serial Port initialization failed \n")
            rate.sleep()
            pass
    rospy.loginfo("=================================start plan!!! \n")
    #建立环境
    robot.create_scene(0.2815,0.2)# 创建障碍物
    #robot.create_scene(0.15,0.1)
    rospy.sleep(1)
    #获取box_姿态，路径规划
    sub = rospy.Subscriber("box_pose_in_world",move_pose_steps,robot.callback)
    #修改当前robot姿态
    pub_joint_state = rospy.Publisher("change_joint_state",JointState,queue_size=10)
    rospy.spin()