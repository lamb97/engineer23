#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

ajointstate=JointState()
ajointstate.name = ['car_fb2car_yaw', 'car_lr2car_fb','fb_move2roll_link','origin_global2car_lr','pitch_link2yaw_link','roll_link2pitch_link','ud_fixed2ud_move','ud_move2fb_move']
ajointstate.position = [0,0,0,0,0,0,0,0]
def callback(jointstate):
    ajointstate.name=jointstate.name
    ajointstate.position=jointstate.position

if __name__ == '__main__':
    rospy.init_node('pub_joint_state', anonymous=True)
    pub = rospy.Publisher("/joint_states",JointState, queue_size=1)
    change_state_subscriber = rospy.Subscriber("change_joint_state",JointState,callback,buff_size=10)

    while(not rospy.is_shutdown()):
        ajointstate.header.stamp=rospy.Time.now()
        pub.publish(ajointstate)

    rospy.spin()
 