//action server
# include <ros/ros.h>
# include <actionlib/server/simple_action_server.h>
# include <control_msgs/FollowJointTrajectoryAction.h>
# include <std_msgs/Float32MultiArray.h>
# include <iostream>
# include <moveit_msgs/RobotTrajectory.h>
//串口发送
#include <ros/ros.h>
#include "serial/serial.h"
using namespace std;

serial::Serial serial_stm;
typedef union
{
	float data;
	unsigned char data8[4];
} data_u;
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;// 重命名类型为 Server
void execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goalPtr, Server* moveit_server)
{
    int n_joints = goalPtr->trajectory.joint_names.size();
    int n_points = goalPtr->trajectory.points.size();
     // 一次
    for(int i=0; i<n_points; i++)
    {   
        unsigned char serial_tf_data[200];
        uint8_t serial_tf_data_num=0;
        for(int j=0;j<n_joints; j++) // 遍历每组路点中的每个关节数据
        {
            float p = goalPtr->trajectory.points[i].positions[j];
            float v = goalPtr->trajectory.points[i].velocities[j];
            float a = goalPtr->trajectory.points[i].accelerations[j];
            // cout <<"points:"<<i+1<<" joints:"<<j+1<<" p:"<<p<<" v:"<<v<<" a:"<<a<<endl;
            data_u float2u8_data;
            float2u8_data.data=p;
            serial_tf_data[serial_tf_data_num++]=float2u8_data.data8[0];
            serial_tf_data[serial_tf_data_num++]=float2u8_data.data8[1];
            serial_tf_data[serial_tf_data_num++]=float2u8_data.data8[2];
            serial_tf_data[serial_tf_data_num++]=float2u8_data.data8[3];
            float2u8_data.data=v;
            serial_tf_data[serial_tf_data_num++]=float2u8_data.data8[0];
            serial_tf_data[serial_tf_data_num++]=float2u8_data.data8[1];
            serial_tf_data[serial_tf_data_num++]=float2u8_data.data8[2];
            serial_tf_data[serial_tf_data_num++]=float2u8_data.data8[3];
            float2u8_data.data=a;
            serial_tf_data[serial_tf_data_num++]=float2u8_data.data8[0];
            serial_tf_data[serial_tf_data_num++]=float2u8_data.data8[1];
            serial_tf_data[serial_tf_data_num++]=float2u8_data.data8[2];
            serial_tf_data[serial_tf_data_num++]=float2u8_data.data8[3];
        }
        if(i==n_points-1)
        {
            serial_tf_data[serial_tf_data_num++]=0xdd;
            serial_tf_data[serial_tf_data_num++]=0xaa;
        }
        else
        {
            serial_tf_data[serial_tf_data_num++]=0x0d;
            serial_tf_data[serial_tf_data_num++]=0x0a;
        }
        serial_stm.write(serial_tf_data,serial_tf_data_num);
    }
    ROS_INFO("=============================plan has been sent to STM32 \n");
    moveit_server->setSucceeded(); 
}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"moveit_action_server");
    ros::NodeHandle nh;
    //定义串口
    serial_stm.setPort("/dev/ttyUSB0");// 这个端口号就是之前用cutecom看到的端口名称
    serial_stm.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    serial_stm.setTimeout(to); 
    // 创建 action 对象(NodeHandle，话题名称，回调函数解析传入的目标值，服务器是否自启动)
    Server moveit_server(nh,"robot_controller/follow_joint_trajectory", boost::bind(&execute_callback, _1, &moveit_server), false);
    // 手动启动服务器
    moveit_server.start();
    ROS_INFO("=============================action server starting, ends \n");
    //启动串口
    ros::Rate rate(0.5); 
    while(ros::ok())
    {
        try
        {
            serial_stm.open();         //打开串口
            ROS_INFO("=============================Serial Port initialization, ends \n");
            break;
        }
        catch(serial::IOException &e)
        {
            ROS_ERROR("=============================Serial Port initialization failed \n");
            rate.sleep();
        }
    }
    ros::spin();
    return 0;
}
