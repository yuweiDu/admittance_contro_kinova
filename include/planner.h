// import numpy as np
// import sys
// import jaco_control.utils.trajectory as trajectory
// import copy
// import matplotlib.pyplot as plt

// # ROS libs
// import rospy
// import moveit_commander
// import tf.transformations

// # ROS messages
// from geometry_msgs.msg import Pose, PoseStamped
// from sensor_msgs.msg import JointState
// from moveit_msgs.msg import RobotState
// from std_msgs.msg import Header

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>


#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>

using namespace Eigen;
typedef Matrix<double, 6, 1> Vector6d;

class Planner{
private:
    ros::NodeHandle nh_;
    std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface move_group_;
    Vector6d start_pose, end_pose;
    std::vector<geometry_msgs::Pose> waypoints;

public:
    Planner();
    void plan_moveit(Vector3d position, Vector4d orientation, bool eular_flag, float time_scale);
    void diff_traj();

};