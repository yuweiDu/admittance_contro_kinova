#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
#include <admittance_control/Admittance.h>
// #include "geometry_msgs/WrenchStamped.h"
// #include "geometry_msgs/TwistStamped.h"



Admittance::Admittance(ros::NodeHandle &n, //constructor of class Admittance
    double frequency,
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
    std::vector<double> desired_velocity) :
  nh_(n), loop_rate_(frequency),
  M_(M.data()), D_(D.data()),K_(K.data()),desired_pose_(desired_pose.data()),
  arm_max_vel_(arm_max_vel), arm_max_acc_(arm_max_acc),
  base_link_(base_link), end_link_(end_link),rotation_ft_end_(rotation_ft_end.data()), rotation_gripper_ft_(rotation_gripper_ft.data()), desired_vel_(desired_velocity.data()){

  //* Subscribers
//   sub_pose_arm_         = nh_.subscribe(topic_arm_pose, 5, &Admittance::pose_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());

  sub_velocity_joint_   = nh_.subscribe(topic_joint_velocity, 5, &Admittance::velocity_joint_callback, this,ros::TransportHints().reliable().tcpNoDelay());

  sub_wrench_state_     = nh_.subscribe(topic_wrench_state, 5, &Admittance::state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  
//   //* Publishers
//   pub_arm_cmd_              = nh_.advertise<kinova_msgs::PoseVelocity>(topic_arm_command, 5);

  // initializing the class variables
  arm_position_.setZero();
  arm_twist_.setZero();
  wrench_external_.setZero();
  desired_pose_position_ << desired_pose_.topRows(3);
  desired_pose_orientation_.coeffs() << desired_pose_.bottomRows(4)/desired_pose_.bottomRows(4).norm();



  //*  KDL init to compute Jacobian
  std::string robot_desc_string;
  nh_.param("robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree)){
    ROS_ERROR("Failed to construct kdl tree");
    // return false;
  }
  
  kdl_tree.getChain(base_link_,end_link_,kdl_chain_);
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  q_.resize(kdl_chain_.getNrOfJoints());
  qdot_.resize(kdl_chain_.getNrOfJoints());
  Jac_.resize(kdl_chain_.getNrOfJoints());
  std::cout<<"joint number is"<<kdl_chain_.getNrOfJoints()<<std::endl;


/////////////////////////////////////////////////////////////////////////////////////////////////////
//   while (nh_.ok() && !arm_position_(0)) {
//     ROS_WARN_THROTTLE(1, "Waiting for the state of the arm...");
//     ros::spinOnce();
//     loop_rate_.sleep();
//     // std::cout<<arm_position_(0)<<std::endl;
//   }
  ROS_INFO("DYW");
/////////////////////////////////////////////////////////////////////////////////////////////////////


  // Init integrator
  arm_desired_twist_adm_.setZero();


  ft_arm_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;

  wait_for_transformations();
}

void Admittance::wait_for_transformations() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  // Makes sure all TFs exists before enabling all transformations in the callbacks
  // while (!get_rotation_matrix(rot_matrix, listener, "world", base_link_)) {sleep(1);}
  base_world_ready_ = true;
  // while (!get_rotation_matrix(rot_matrix, listener, base_link_, "world")) {sleep(1);}
  world_arm_ready_ = true;
  while (!get_rotation_matrix(rot_matrix, listener, base_link_, end_link_)) {sleep(1);}
  ft_arm_ready_ = true;
  ROS_INFO("The Force/Torque sensor is ready to use.");
}


bool Admittance::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try {
    listener.lookupTransform(from_frame, to_frame,
                            ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) {
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }
  return true;
}


void Admittance::pose_arm_callback(const geometry_msgs::PoseStampedConstPtr pose){
    arm_position_ << pose->pose.position.x,
                    pose->pose.position.y,
                    pose->pose.position.z;
    // std::cout<<arm_position_<<std::endl;

    arm_orientation_.coeffs() << pose->pose.orientation.x,
                                 pose->pose.orientation.y,
                                 pose->pose.orientation.z,
                                 pose->pose.orientation.w;

}

void Admittance::velocity_joint_callback(const sensor_msgs::JointStateConstPtr joint_state){
    // q_.data<<joint_state->position[0],
    //          joint_state->position[1],
    //          joint_state->position[2],
    //          joint_state->position[3],
    //          joint_state->position[4],
    //          joint_state->position[5];
    // q_.data(0)= joint_state->position[0];
            //  joint_state->position[1],
            //  joint_state->position[2],
            //  joint_state->position[3],
            //  joint_state->position[4],
            //  joint_state->position[5];

    // qdot_.data<<joint_state->velocity[0],
    //           joint_state->velocity[1],
    //           joint_state->velocity[2],
    //           joint_state->velocity[3],
    //           joint_state->velocity[4],
    //           joint_state->velocity[5];

    // jnt_to_jac_solver_->JntToJac(q_,Jac_);
    // arm_twist_<<Jac_.data * qdot_.data;
    
}


//* externel wrench mesaured by the force sensor
void Admittance::state_wrench_callback(
  // const force_sensor::ForceConstPtr msg) {
    const geometry_msgs::WrenchStampedConstPtr msg) {
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;
  if (ft_arm_ready_) {
    wrench_ft_frame <<  msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z,0,0,0; //original
    // wrench_ft_frame <<  msg->wrench.force.z,
    //                     msg->wrench.force.y,
    //                     msg->wrench.force.x,
    //                     msg->wrench.torque.z,
    //                     msg->wrench.torque.y,
    //                     msg->wrench.torque.x;
    // wrench_ft_frame << msg->Fx,msg->Fy,msg->Fz,0,0,0; //6-axis sensor

    float force_thres_lower_limit_ = 3;
    float force_thres_upper_limit_ = 100;
    if(fabs(wrench_ft_frame(0)) < force_thres_lower_limit_ || fabs(wrench_ft_frame(0)) > force_thres_upper_limit_){wrench_ft_frame(0) = 0;}
    else{
      if(wrench_ft_frame(0) > 0){wrench_ft_frame(0) -= force_thres_lower_limit_;}
      else{wrench_ft_frame(0) += force_thres_lower_limit_;}
    }
    if(fabs(wrench_ft_frame(1)) < force_thres_lower_limit_ || fabs(wrench_ft_frame(1)) > force_thres_upper_limit_){wrench_ft_frame(1) = 0;}
    else{
      if(wrench_ft_frame(1) > 0){wrench_ft_frame(1) -= force_thres_lower_limit_;}
      else{wrench_ft_frame(1) += force_thres_lower_limit_;}
    }
    if(fabs(wrench_ft_frame(2)) < force_thres_lower_limit_ || fabs(wrench_ft_frame(2)) > force_thres_upper_limit_){wrench_ft_frame(2) = 0;}
    else{
      if(wrench_ft_frame(2) > 0){wrench_ft_frame(2) -= force_thres_lower_limit_;}
      else{wrench_ft_frame(2) += force_thres_lower_limit_;}
    }

    get_rotation_matrix(rotation_ft_base, listener_ft_, base_link_, end_link_);
    std::cout<<rotation_ft_base<<std::endl;
    wrench_external_ <<  rotation_ft_base * wrench_ft_frame;
  }
}


void Admittance::compute_admittance() {
  ros::Duration duration = loop_rate_.expectedCycleTime();

  //* position error
  error.topRows(3) = arm_position_ - desired_pose_position_;

  //* orientation error
  if(desired_pose_orientation_.coeffs().dot(arm_orientation_.coeffs()) < 0.0)
  {
    arm_orientation_.coeffs() << -arm_orientation_.coeffs();
  }
  Eigen::Quaterniond quat_rot_err (arm_orientation_ * desired_pose_orientation_.inverse());
  if(quat_rot_err.coeffs().norm() > 1e-3)
  {
    quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();


 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //* translation velocity error
  // derror.topRows(3) = arm_twist_.topRows(3) - desired_trans_vel_;
  //* angular velocity error

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Translation error w.r.t. desired equilibrium
  Vector6d coupling_wrench_arm;

  coupling_wrench_arm=  D_ * (arm_desired_twist_adm_) + K_*error;
  arm_desired_accelaration = M_.inverse() * ( - coupling_wrench_arm  + wrench_external_); // why not error of acceleration rather than desired acc

  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();

  if (a_acc_norm > arm_max_acc_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!"
                             << " norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }
  // Integrate for velocity based interface
  // ros::Duration duration = loop_rate_.expectedCycleTime();
  arm_desired_twist_adm_ += arm_desired_accelaration * duration.toSec();
}

void Admittance::run() {

  ROS_INFO("Running the admittance control loop .................");

  while (nh_.ok()) {

    compute_admittance();

    // send_commands_to_robot();

    ros::spinOnce();
    loop_rate_.sleep();
  }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "admittance_node");

    ros::NodeHandle nh;
    double frequency;// = 100.0;

    // Parameters
    // std::string topic_arm_state;
    std::string topic_arm_pose;
    std::string topic_joint_velocity;
    std::string topic_arm_command;
    std::string topic_wrench_state;
    std::string base_link;
    std::string end_link;

    std::vector<double> M;
    std::vector<double> D;
    std::vector<double> K;
    std::vector<double> desired_pose;
    
    double arm_max_vel;
    double arm_max_acc;

    std::vector<double> rotation_ft_end;
    std::vector<double> rotation_gripper_ft;

    std::vector<double> desired_velocity;
    

    // LOADING PARAMETERS FROM THE ROS SERVER 

    // Topic names
    // if (!nh.getParam("topic_arm_state", topic_arm_state)) { ROS_ERROR("Couldn't retrieve the topic name for the state of the arm."); return -1; }
    if (!nh.getParam("topic_arm_pose", topic_arm_pose)) { ROS_ERROR("Couldn't retrieve the topic name for the state of the arm."); return -1; }
    if (!nh.getParam("topic_joint_velocity", topic_joint_velocity)) { ROS_ERROR("Couldn't retrieve the topic name for the velocity of the joints."); return -1; }
    if (!nh.getParam("topic_arm_command", topic_arm_command)) { ROS_ERROR("Couldn't retrieve the topic name for commanding the arm."); return -1; }
    if (!nh.getParam("topic_wrench_state", topic_wrench_state)) { ROS_ERROR("Couldn't retrieve the topic name for the force/torque sensor."); return -1; }
    // ADMITTANCE PARAMETERS
    if (!nh.getParam("mass_arm", M)) { ROS_ERROR("Couldn't retrieve the desired mass of the arm."); return -1; }
    if (!nh.getParam("damping_arm", D)) { ROS_ERROR("Couldn't retrieve the desired damping of the coupling."); return -1; }
    if (!nh.getParam("stiffness_coupling", K)) { ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling."); return -1; }
    if (!nh.getParam("base_link", base_link)) { ROS_ERROR("Couldn't retrieve the base_link."); return -1; }
    if (!nh.getParam("end_link", end_link)) { ROS_ERROR("Couldn't retrieve the end_link."); return -1; } 
    if (!nh.getParam("desired_pose", desired_pose)) { ROS_ERROR("Couldn't retrieve the desired pose of the spring."); return -1; }
    if (!nh.getParam("arm_max_vel", arm_max_vel)) { ROS_ERROR("Couldn't retrieve the max velocity for the arm."); return -1;}
    if (!nh.getParam("arm_max_acc", arm_max_acc)) { ROS_ERROR("Couldn't retrieve the max acceleration for the arm."); return -1;}
    if (!nh.getParam("frequency", frequency)) { ROS_ERROR("Couldn't retrieve the max acceleration for the arm."); return -1;}
    //rotation matrix
    if (!nh.getParam("rotation_ft_end", rotation_ft_end)) { ROS_ERROR("Couldn't retrieve the rotation_ft_end"); return -1;}
    if (!nh.getParam("rotation_gripper_ft", rotation_gripper_ft)) { ROS_ERROR("Couldn't retrieve the rotation_gripper_ft"); return -1;}

    if (!nh.getParam("desired_velocity", desired_velocity)) { ROS_ERROR("Couldn't retrieve the desired_velocity"); return -1;}
    // Constructing the controller
    Admittance admittance(
        nh,
        frequency,
        // topic_arm_state,
        topic_arm_pose,
        topic_joint_velocity,
        topic_arm_command,
        topic_wrench_state,
        M, D, K, desired_pose,
        base_link,
        end_link,
        arm_max_vel,
        arm_max_acc,
        rotation_ft_end,
        rotation_gripper_ft,
        desired_velocity);
    
    ros::spin();

}