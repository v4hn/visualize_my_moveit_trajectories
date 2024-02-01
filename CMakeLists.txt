cmake_minimum_required(VERSION 3.0.2)
project(visualize_my_moveit_trajectories)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rospy
)
