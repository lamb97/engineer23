cmake_minimum_required(VERSION 3.1.3)
project(robot_moveit_config4)



#导入包
find_package(catkin REQUIRED COMPONENTS
)

#A.导入消息文件
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

#B.为功能包生成cmake配置文件
catkin_package()

#C.编译
#增加额外的头文件
# include_directories(include ${catkin_INCLUDE_DIRS} )
#添加可执行文件，并将他们与源文件连接




install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




