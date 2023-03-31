#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"


#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>

using namespace Eigen;
typedef Matrix<double, 6, 1> Vector6d; 
typedef Matrix<double, 9, 1> Vector9d; 

class Trajectory{
private:
  std::vector<geometry_msgs::Pose> waypoints_;
  std::vector<Vector3d> vel_;
  std::vector<Vector3d> acc_;
  Vector3d start_pos_;
  Vector3d end_pos_;
  double duration_;

public:
  Trajectory(Vector3d start_postion, Vector3d end_position, double duration);
  Vector3d poly_1D(double start, double end, double duration, double t);
  Vector9d position_plan(double t);
  
};


Trajectory::Trajectory(Vector3d start_postion, Vector3d end_position, double duration)
:start_pos_(start_postion), end_pos_(end_position), duration_(duration){

}

Vector3d Trajectory::poly_1D(double p_start, double p_end, double duration, double t){
  float k0,k1,k2,k3;
  k0 = p_start;
  k1 = 0;
  k2 = 3/(duration*duration)*(p_end-p_start);
  k3 = -2/(duration*duration*duration)*(p_end-p_start);

  // std::vector<double> p;
  // std::vector<double> dp;
  // std::vector<double> ddp;
  // p.push_back(k0+k2*t*t+k3*t*t*t);
  // dp.push_back(2*k2*t+3*k3*t*t);
  // ddp.push_back(2*k2+6*k3*t);

  // std::vector<Vector3d> traj;
  Vector3d temp;
  temp << k0+k2*t*t+k3*t*t*t,
          2*k2*t+3*k3*t*t,
          2*k2+6*k3*t;
  // traj.push_back(temp);
  return temp;
}

Vector9d Trajectory::position_plan(double t){
  Vector3d x,y,z;
  x = poly_1D(start_pos_(0),end_pos_(0), duration_, t);
  y = poly_1D(start_pos_(1),end_pos_(1), duration_, t);
  z = poly_1D(start_pos_(2),end_pos_(2), duration_, t);

  Vector9d temp;
  temp << x(0),y(0),z(0),x(1),y(1),z(1),x(2),y(2),z(2);
  return temp;
}