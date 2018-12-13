// Jaing Xu
// jiangxvv@sjtu.edu.cn

// basic class of controller

#include "basic_controller.h"
#include<cmath>

BasicController::BasicController(const VesselControlProperty& vessel, const double& step_size) : step_size_(step_size)
{
    mass_ = vessel.mass;
    radius_=vessel.radius;
    Irr_ = mass_*radius_*radius_;
    a11_ = vessel.a11;
    a22_ = vessel.a22;
    a23_ = vessel.a23;
    a33_ = vessel.a33;
    xg_ = vessel.xg;
    d11_ = vessel.d11;
    d22_ = vessel.d22;
    d33_ = vessel.d33;
}

// rotation matrix definition
Eigen::Matrix3d BasicController::RotationMatrix(const Eigen::Vector3d& pose) const
{
    Eigen::Matrix3d rz;
    rz << cos(pose[2]), -sin(pose[2]), 0, sin(pose[2]), cos(pose[2]), 0, 0, 0, 1;
    return rz;
}

// inertia matrix of the vehicle
Eigen::Matrix3d BasicController::InertiaMatrix() const
{
    Eigen::Matrix3d inertia_mat;
    inertia_mat << mass_ + a11_, 0, 0, 0, mass_ + a22_, a23_, 0, a23_,
    Irr_ + a33_;
    return inertia_mat;
}

// inertial centripetal and coriolis matrix
Eigen::Matrix3d BasicController::InertiaCCMatrix(const Eigen::Vector3d& vec) const
{
    Eigen::Matrix3d inertia_cc_mat;
    inertia_cc_mat << 0, -mass_* vec[2], -a23_* vec[2] - a22_* vec[1],
    mass_* vec[2], 0, a11_* vec[0], a23_* vec[2] + a22_* vec[1],
    -a11_* vec[0], 0;
    return inertia_cc_mat;
}


// damping matrix of the vehicle
Eigen::Vector3d BasicController::DampingVector(const Eigen::Vector3d& vec) const
{
  double rou=1025;
  double l=95;
  double u=vessel_velocity_[0];
  double v=vessel_velocity_[1];
  double uu=sqrt(pow(v,2)+pow(u,2));
  Eigen::Vector3d damping1, damping2;
  double d11=d11_*rou/2*uu*pow(l,2);
  double d22=d22_*rou/2*uu*pow(l,2);
  double d33=d33_*rou/2*uu*pow(l,4);
  damping1 << d11* vec[0], d22* vec[1] ,  d33* vec[2];

  // Xu|u| is different in forward and reverse conditions
  //double Xuu = (vec[0] > 0) ? 112.4 : 198.1;
  //damping2 << Xuu* vec[0] * std::fabs(vec[0]), 0, 0;
  return damping1;
}
