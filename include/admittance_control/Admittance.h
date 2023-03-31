/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:08:00 
 * @Last Modified by: MingshanHe
 * @Last Modified time: 2021-12-05 04:08:21
 * @Licence: MIT Licence
 */

#ifndef ADMITTANCE_H
#define ADMITTANCE_H

#include "ros/ros.h"

// MoveIt
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>

#include "math.h"
#include "force_sensor/Force.h"
#include "kinova_msgs/KinovaPose.h"
#include "kinova_msgs/PoseVelocity.h"
#include "kinova_msgs/JointVelocity.h"
#include "sensor_msgs/JointState.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/jacobian.hpp"
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
// #include "cartesian_state_msgs/PoseTwist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

#include <memory>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <cmath>


#include "planner.h"
#include "trajectory.h"

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
#define PI 3.1415926

class Admittance
{
protected:
  // ROS VARIABLES:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;

  // ADMITTANCE PARAMETERS:
  Matrix6d M_, D_, K_;

  // Subscribers:
  ros::Subscriber sub_arm_state_;
  ros::Subscriber sub_pose_arm_;
  ros::Subscriber sub_velocity_joint_;
  ros::Subscriber sub_wrench_state_;
  // Publishers:
  ros::Publisher pub_arm_cmd_;

  // Variables:
  Vector3d      arm_position_;
  Vector3d      euler_;
  Quaterniond   arm_orientation_;
  Vector6d      arm_twist_;
  Vector6d      wrench_external_;
  Vector6d      arm_desired_twist_adm_;
  Vector6d      arm_desired_accelaration;

  Vector7d      desired_pose_;
  Vector3d      desired_pose_position_;
  Quaterniond   desired_pose_orientation_;
  // Vector3d      desired_trans_vel_;
  // Vector3d      desired_trans_acc_;

  Vector6d      desired_vel_;
  Vector6d      desired_acc_;

  Vector6d      error;
  Vector6d      derror;

  // KDL::Tree     mytree;
  // KDL::Jacobian Jac_;
  // urdf::Model   mymodel;

  // TF:
  // Transform from base_link to world
  Matrix6d rotation_base_;
  // Listeners
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;

  // Guards
  bool ft_arm_ready_;
  bool base_world_ready_;
  bool world_arm_ready_;

  double arm_max_vel_;
  double arm_max_acc_;

public:
  Admittance(ros::NodeHandle &n, double frequency,
                      std::string topic_arm_pose,
                      std::string topic_joint_velocity,
                      std::string topic_arm_command,
                      std::string topic_wrench_state,
                      std::vector<double> M,
                      std::vector<double> D,
                      std::vector<double> K,
                      std::vector<double> desired_pose,
                      std::string base_link,
                      std::string end_link,
                      double arm_max_vel,
                      double arm_max_acc,
                      std::vector<double> rotation_ft_end,
                      std::vector<double> rotation_gripper_ft,
                      std::vector<double> desired_velocity
                       );
  ~Admittance(){}
  void run();
private:
  // Control
  void compute_admittance();
  // Callbacks
  // void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);
  void pose_arm_callback(const geometry_msgs::PoseStampedConstPtr pose);
  void velocity_joint_callback(const sensor_msgs::JointStateConstPtr joint_state);
  void state_wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg);
  // void state_wrench_callback(const force_sensor::ForceConstPtr msg);
  // 
  void send_commands_to_robot();

  // 
  void wait_for_transformations();
  bool get_rotation_matrix(Matrix6d & rotation_matrix,
                           tf::TransformListener & listener,
                           std::string from_frame,  std::string to_frame);
private:
  std::string   base_link_;
  std::string   end_link_;

  KDL::Tree    kdl_tree;
  KDL::JntArray q_;
  KDL::JntArray qdot_;
  KDL::Jacobian Jac_;
  // The chain of links and joints  
  KDL::Chain kdl_chain_;

  // KDL Solvers performing the actual computations                                                                                                                               
  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
  float gripper_mass;
  Matrix3d rotation_ft_end_;
  Matrix3d rotation_gripper_ft_;
};

#endif // ADMITTANCE_H

