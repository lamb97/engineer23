#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>    //catkin_package导入了
// PCL 的相关的头文件
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>  
using namespace std;
#include "robot_control/get_pointcloud_3d.h"


bool doreq(robot_control::get_pointcloud_3d::Request& req,robot_control::get_pointcloud_3d::Response& resp)
{   
  //sensor_msgs中的pointcould2 转化为 pcl的pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_msg(new pcl::PointCloud<pcl::PointXYZ>); 
  pcl::fromROSMsg((req.pointcloud), *pcl_cloud_msg); 

  // cout<<(*pcl_cloud_msg).width<<endl;
  // cout<<(*pcl_cloud_msg).height<<endl;
  // cout<<(*pcl_cloud_msg).points.size()<<endl;
  //点云点，转换为PointStamped格式
  pcl::PointXYZ apointxyz=(*pcl_cloud_msg).at(req.x,req.y);
  resp.x=apointxyz.x;
  resp.y=apointxyz.y;
  resp.z=apointxyz.z;
  // cout<<"位置"<<req.x<<","<<req.y<<"\n转换前坐标"<<apointxyz<<endl;
  //cout<<"点云的frame"<<(*pcl_cloud_msg).header;
  return true;
}

int main (int argc, char** argv)
{
  // 初始化 ROS节点
  ros::init (argc, argv, "get_pointcloud_3d_service");
  ros::NodeHandle nh;  
  
  // 为接受点云数据创建一个订阅节点
  ros::ServiceServer server = nh.advertiseService ("get_pointcloud_3d_service", doreq);
  // 回调
  ros::spin ();
}
