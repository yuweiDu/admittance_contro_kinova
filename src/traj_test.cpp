/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:08:47 
 * @Last Modified by:   MingshanHe 
 * @Last Modified time: 2021-12-05 04:08:47 
 * @Licence: MIT Licence
 */
#include <admittance_control/Admittance.h>
// #include "trajectory.h"


// Vector3d start(0,0.30,0.05),end(0,0.50,0.05);
// Vector3d start_r(0,0.50,0.05),end_r(0,0.30,0.05);

// double duration = 3.;
// Trajectory traj(start, end, duration);
// Trajectory traj_r(start_r,end_r,duration);

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "jacoTraj");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  
  double duration;
  double velocity;
  n.getParam("duration",duration);
  n.getParam("velocity",velocity);

  ros::Publisher chatter_pub = n.advertise<kinova_msgs::PoseVelocity>("/j2s6s300_driver/in/cartesian_velocity", 1000);

  ros::Rate loop_rate(100);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int cnt = 0;
  double dt = loop_rate.expectedCycleTime().toSec();

  while (n.ok()) {
    while(cnt<duration/dt){
      Vector9d pva;
      // pva = traj.position_plan(dt*cnt);

      kinova_msgs::PoseVelocity arm_twist_cmd;
      arm_twist_cmd.twist_linear_x  = pva(3);
      arm_twist_cmd.twist_linear_y  = velocity;//pva(4);
      arm_twist_cmd.twist_linear_z  = pva(5);
      arm_twist_cmd.twist_angular_x = 0.;
      arm_twist_cmd.twist_angular_y = 0.;
      arm_twist_cmd.twist_angular_z = 0.;

      chatter_pub.publish(arm_twist_cmd);
      ros::spinOnce();
      loop_rate.sleep();
      ++cnt;
    }
    
    cnt = 0;
    while (cnt<duration/dt) {
      Vector9d pva;
      // pva = traj_r.position_plan(dt*cnt);

      kinova_msgs::PoseVelocity arm_twist_cmd;
      arm_twist_cmd.twist_linear_x  = pva(3);
      arm_twist_cmd.twist_linear_y  = -velocity;//pva(4);
      arm_twist_cmd.twist_linear_z  = pva(5);
      arm_twist_cmd.twist_angular_x = 0.;
      arm_twist_cmd.twist_angular_y = 0.;
      arm_twist_cmd.twist_angular_z = 0.;

      chatter_pub.publish(arm_twist_cmd);
      ros::spinOnce();
      loop_rate.sleep();
      ++cnt;
    }
    cnt = 0;
  }

  return 0;
}