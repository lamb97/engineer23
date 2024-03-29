#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

#判断两个list是否接近在一定范围
def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class my_moveit_ik:
    def __init__(self):
        group_name = "robot"
        eef_link_name = "hand"
        #初始化node和moveit_commander,sys.argv可以命令行输入参数
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        #建立robot控制/信息获取模型
        robot = moveit_commander.RobotCommander()
        
        #设置/获取环境信息
        scene = moveit_commander.PlanningSceneInterface()
        scene.clear()


        tool_length=0.15
        tool_size = [tool_length, tool_length, tool_length]
        tool_pose = geometry_msgs.msg.PoseStamped()
        tool_pose.pose.position.x = 0.0
        tool_pose.pose.position.y = 0.0
        tool_pose.pose.position.z = -tool_size[2] / 2.0 
        tool_pose.pose.orientation.x = 1e-6
        tool_pose.pose.orientation.y = 1e-6
        tool_pose.pose.orientation.z = 1e-6
        tool_pose.pose.orientation.w = 1.000000

        tool_pose.header.frame_id = eef_link_name
        scene.attach_box(eef_link_name, 'box', tool_pose, tool_size)

        table_size = [0.2, 0.2, 0.01]
        table_ground = 0.5
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = 'world'
        table_pose.pose.position.x = 2.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground
        table_pose.pose.orientation.x = 1e-6
        table_pose.pose.orientation.y = 1e-6
        table_pose.pose.orientation.z = 1e-6
        table_pose.pose.orientation.w = 1.000000
        scene.add_box('table1', table_pose, table_size)

        table_size = [0.2, 0.2, 0.01]
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = 'world'
        table_pose.pose.position.x = 2.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[0]
        table_pose.pose.orientation.x = 1e-6
        table_pose.pose.orientation.y = 1e-6
        table_pose.pose.orientation.z = 1e-6
        table_pose.pose.orientation.w = 1.000000
        scene.add_box('table2', table_pose, table_size)

        table_size = [0.2, 0.01, 0.2]
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = 'world'
        table_pose.pose.position.x = 2.0
        table_pose.pose.position.y = -0.1
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.x = 1e-6
        table_pose.pose.orientation.y = 1e-6
        table_pose.pose.orientation.z = 1e-6
        table_pose.pose.orientation.w = 1.000000
        scene.add_box('table3', table_pose, table_size)

        table_size = [0.2, 0.01, 0.2]
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = 'world'
        table_pose.pose.position.x = 2.0
        table_pose.pose.position.y = 0.1
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.x = 1e-6
        table_pose.pose.orientation.y = 1e-6
        table_pose.pose.orientation.z = 1e-6
        table_pose.pose.orientation.w = 1.000000
        scene.add_box('table4', table_pose, table_size)

        table_size = [0.01, 0.2, 0.2]
        table_ground = 0.5
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = 'world'
        table_pose.pose.position.x = 2.0+table_size[2] / 2.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.x = 1e-6
        table_pose.pose.orientation.y = 1e-6
        table_pose.pose.orientation.z = 1e-6
        table_pose.pose.orientation.w = 1.000000
        scene.add_box('table5', table_pose, table_size)


        #规划 执行路径
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(eef_link_name)
        move_group.set_max_velocity_scaling_factor(1.0)
        move_group.set_max_acceleration_scaling_factor(1.0)
        move_group.set_goal_position_tolerance(0.05)
        move_group.set_goal_orientation_tolerance(0.05)
        move_group.allow_replanning(True)
        move_group.set_planning_time(10)

        #用于rviz显示
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        #输出信息验证，并设置参数
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        self.planning_frame = planning_frame #world
        self.eef_link = eef_link  #robot group中的是yaw_Link
        self.group_names = group_names #robot

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher

    #使末端到达pose
    def get_go_to_pose_goal_plan(self,pose_goal):
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_pose_target(pose_goal)
        _,path_plan,_,error_code = self.move_group.plan()

        if error_code.val==1:
            print("============ path plan\n",path_plan)
        else:
            print("============ path plan error code val",error_code.val)
        return path_plan,error_code.val
        
    def execute_plan(self, plan):
        p=self.move_group.get_current_pose(end_effector_link=self.eef_link)
        print("============ end effector pose before\n",p)

        self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        p=self.move_group.get_current_pose(end_effector_link=self.eef_link)
        print("============ end effector pose after\n",p)
        # p=self.move_group.get_current_pose(end_effector_link="yaw_Link")
        # print("============ :end effector pose before",p)

def demo():
    my_robot=my_moveit_ik()

    pose_goal = geometry_msgs.msg.Pose()    #其中的orientation要满足quaternion的归一，要准确/精度高
    #0.2框位置
    pose_goal.orientation.x = 0.06387169937892422
    pose_goal.orientation.y = -0.6993020431199148
    pose_goal.orientation.z = -0.012408384949972562
    pose_goal.orientation.w = 0.7118588978791333
    pose_goal.position.x = 1.8936536955324288
    pose_goal.position.y = 0.0014844789487815568
    pose_goal.position.z = 0.5972337013953857
    # 0.3框位置
    # pose_goal.orientation.x = 0.05402249700867762
    # pose_goal.orientation.y = -0.6899472953809638
    # pose_goal.orientation.z = -0.03977392259991941
    # pose_goal.orientation.w = 0.7207442920304371
    # pose_goal.position.x = 1.859313994012385
    # pose_goal.position.y = 0.007528259573057781
    # pose_goal.position.z = 0.7630417665667684
    #假设的框位置
    # pose_goal.orientation.x = 1.4105522192170936e-06
    # pose_goal.orientation.y = -0.6325967163556954
    # pose_goal.orientation.z = 0.03304217603119068
    # pose_goal.orientation.w = 0.7737762008857121
    # pose_goal.position.x = 1.4367495259645042
    # pose_goal.position.y = -0.0008209238474464545
    # pose_goal.position.z = 0.7676040085212564
    #起始位置
    # pose_goal.orientation.x = 0
    # pose_goal.orientation.y = 0
    # pose_goal.orientation.z = 0
    # pose_goal.orientation.w = 1
    # pose_goal.position.x =    0
    # pose_goal.position.y =    0
    # pose_goal.position.z =    0.46
    #普通位置
    # pose_goal.orientation.x = 0.12971536735824005
    # pose_goal.orientation.y = -0.1397241609450597
    # pose_goal.orientation.z = -0.4795230244804937
    # pose_goal.orientation.w = 0.8565680074065314
    # pose_goal.position.x =    1.4019208377451786
    # pose_goal.position.y =    -1.1918053385294118
    # pose_goal.position.z =    0.7054052524813217

    path_plan,error_code_val=my_robot.get_go_to_pose_goal_plan(pose_goal)

    if error_code_val==1:
        my_robot.execute_plan(path_plan)

if __name__ == "__main__":
    demo()
    #my_robot=my_moveit_ik()