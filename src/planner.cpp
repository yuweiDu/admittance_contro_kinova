#include "planner.h"

Planner::Planner():move_group_("arm"){
  // 
  // Initializes the inverse kinematics system. Required if inverse kinematics is going to be used.
  // :return None
  // 
  // set up MoveIt! robot commander and move moveit_group commander
  // set up MoveIt! moveit_scene interface
  // move_group_("arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // setting the position and orientation tolerance
  move_group_.setGoalPositionTolerance(0.001);
  move_group_.setGoalOrientationTolerance(0.001);

  // Add a table to the moveit_scene acting as a ground
  sleep(1);
  
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2.4;
  primitive.dimensions[1] = 2.4;
  primitive.dimensions[2] = 0.3;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.;
  box_pose.position.y = 0.;
  box_pose.position.z = -.03 / 2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  ROS_INFO("MoveIt! init successful.");
}

void Planner::plan_moveit(Vector3d position, Vector4d orientation, bool eular_flag, float time_scale){

  // Plans the motions using the IK implemented in MoveIt!. Note that MoveIt! is not used to execute the plan on the
  // robot, because the controllers are not very useful.
  // :param position: list of cartesian coordinates (XYZ)
  // :type position: list or np.array
  // :param orientation: list of orientation elements (default: quaternion)
  // :type orientation: list or np.array
  // :param start_joint_position: starting joint position for the planner
  // :type: list or np.array
  // :param euler_flag: a flag to indicate whether the orientation is in euler or quaternions
  // :type euler_flag: bool
  // :param time_scale: scales time for the interpolation of raw waypoints
  // :type time_scale: float
  // :return the trajectory with the raw waypoints
  // :rtype: trajectory.Trajectory

  geometry_msgs::Pose pose;
  pose.orientation.x = orientation(0);
  pose.orientation.y = orientation(1);
  pose.orientation.z = orientation(2);
  pose.orientation.w = orientation(3);
  pose.position.x = position(0);
  pose.position.y = position(1);
  pose.position.z = position(2);

  move_group_.setPoseTarget(pose); //  plan a motion for this group to a desired pose for the end-effector:

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group_.clearPoseTargets(); 
  my_plan.trajectory_;

  // geometry_msgs::Pose target_pose3 = start_pose2;
  // target_pose3.position.z -= 0.2;
  waypoints.push_back(pose);  // down
  move_group_.setMaxVelocityScalingFactor(0.1);
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  trajectory.joint_trajectory.points// 是关节的位置速度加速度，不是笛卡尔空间的


  // moveit_msgs::RobotTrajectory trajectory_msg;
  // move_group_.setPlanningTime(10.0);
 
  // double fraction = move_group_.computeCartesianPath(waypoints,
  //                                              0.01,  // eef_step
  //                                              0.0,   // jump_threshold
  //                                              trajectory_msg, 
  //                                              false);
  // // The trajectory needs to be modified so it will include velocities as well.
  // robot_trajectory::RobotTrajectory rt(move_group_.getCurrentState()->getRobotModel(), "arm");  // First to create a RobotTrajectory object
  // rt.setRobotTrajectoryMsg(*move_group_.getCurrentState(), trajectory_msg);   // Second get a RobotTrajectory from trajectory
  // trajectory_processing::IterativeParabolicTimeParameterization iptp; // Thrid create a IterativeParabolicTimeParameterization object
  // success = iptp.computeTimeStamps(rt);// Fourth compute computeTimeStamps
  // ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  // rt.getRobotTrajectoryMsg(trajectory_msg);// Get RobotTrajectory_msg from RobotTrajectory
  // // Finally plan and execute the trajectory
  // my_plan.trajectory_ = trajectory_msg;
  // ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
  // sleep(5.0);
  // move_group_.execute(my_plan);


  



}

void Planner::diff_traj(){

}