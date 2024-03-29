#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
from math import pi, tau, dist, fabs, cos
from robot_control.msg import move_pose_steps
import tf2_ros

class robot_moveit_plan:
    def __init__(self,group_name,eef_link_name):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_end_effector_link(eef_link_name)
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_max_acceleration_scaling_factor(1.0)
        self.move_group.set_goal_position_tolerance(0.05)
        self.move_group.set_goal_orientation_tolerance(0.05)
        self.move_group.allow_replanning(True)
        self.move_group.set_planning_time(15)
        self.move_group.set_num_planning_attempts(2) #设置并行求解个数最大4个

    def print_config(self):
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", group_names)

    def create_scene(self,box_length,tool_length):
        self.scene.clear()

        tool_size = [tool_length, tool_length, tool_length]
        tool_pose = PoseStamped()
        tool_pose.pose.position.x = 0.0
        tool_pose.pose.position.y = 0.0
        tool_pose.pose.position.z = -tool_length / 2
        tool_pose.pose.orientation.x = 1e-6
        tool_pose.pose.orientation.y = 1e-6
        tool_pose.pose.orientation.z = 1e-6
        tool_pose.pose.orientation.w = 1.000000
        tool_pose.header.frame_id = self.move_group.get_end_effector_link()
        self.scene.attach_box(self.move_group.get_end_effector_link(), 'gold', tool_pose, tool_size)


    #使末端到达pose
    def get_go_to_pose_goal_plan(self,pose_goal):
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_pose_target(pose_goal)
        _,path_plan,_,error_code = self.move_group.plan()

        if error_code.val==1:
            rospy.loginfo("=================================path plan ends\n")
            return path_plan
        else:
            rospy.loginfo("=================================path plan error code val %d",error_code.val)
            return None
        
    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)
        rospy.loginfo("=================================plan excuted")
        self.move_group.clear_pose_targets()

    def callback(self,amove_pose_steps):
        while(not rospy.is_shutdown()):
            plan = self.get_go_to_pose_goal_plan(amove_pose_steps.step2)
            if plan:
                break
        self.execute_plan(plan)
        # while(not rospy.is_shutdown()):
        #     plan = self.get_go_to_pose_goal_plan(amove_pose_steps.step2)
        #     if plan:
        #         break
        # self.execute_plan(plan)
        while(not rospy.is_shutdown()):
            pass

if __name__ == "__main__":
    rospy.init_node("get_robot_path", anonymous=True)
    #初始化规划器
    robot=robot_moveit_plan("robot","hand")
    robot.print_config()
    #建立环境
    robot.create_scene(0.2815,0.2)
    #robot.create_scene(0.15,0.1)
    rospy.sleep(1)
    rospy.spin()