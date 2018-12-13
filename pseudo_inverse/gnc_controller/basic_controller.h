// Jaing Xu
// jiangxvv@sjtu.edu.cn

// basic class of controller 

#ifndef BASIC_CONTROLLER_H
#define BASIC_CONTROLLER_H

#include<Eigen/Dense>
#include<string>
#include<iostream>

struct VesselControlProperty
{
    std::string name;
    double mass = 0;
    double radius=0;
   // double Irr = 0;
    double a11 = 0;
    double a22 = 0;
    double a23 = 0;
    double a33 = 0;
    double xg = 0;
    double d11 = 0;
    double d22 = 0;
    double d33 = 0;
};

// class BasicController 
// BasicController is a base class, which defines the basic controller structures

class BasicController
{
protected:
    
    // vessel parameters
  const double GRAVITY_ = 9.81;
  double mass_ = 0; /**< vessel mass */
  double radius_=0;
  double Irr_ = mass_*radius_*radius_;  /**< inertia moment */
  double a11_ = 0;  /**< added mass in surge */
  double a22_ = 0;  /**< added mass in sway */
  double a23_ = 0;  /**< added mass cross term */
  double a33_ = 0;  /**< added mass in yaw */
  double xg_ = 0;   /**< longitudinal coordinate of gravity center */
  double d11_ = 0;  /**< linear damping coefficient in surge */
  double d22_ = 0;  /**< linear damping coefficient in sway */
  double d33_ = 0;  /**< linear damping coefficient in yaw */

  
  // vessel states
  Eigen::Vector3d vessel_position_;     /**< vessel coordinate and heading */
  Eigen::Vector3d vessel_velocity_;     /**< vessel linear velocity and yaw rate */
  Eigen::Vector3d vessel_acceleration_ ; /**< vessel acceleration */
  
  // desire course states
  Eigen::Vector3d course_position_;
  Eigen::Vector3d course_velocity_;  // the derivate of the position in the earth_fixed frame 
  Eigen::Vector3d course_acceleration_;
  
  // state error between the vessel and the course
  Eigen::Vector3d error_pose_, error_vel_, error_acc_;
  
  // control output in surge, sway, yaw
  Eigen::Vector3d output_;
  
  // step size 
  double step_size_=0;
  
  
  // 
  inline Eigen::Matrix3d RotationMatrix(const Eigen::Vector3d& pose) const;
  
  // inertia matrix of the vehicle
  inline Eigen::Matrix3d InertiaMatrix() const;
  
  // inertia centripetal and coriolis matrix of the vehicle
  inline Eigen::Matrix3d InertiaCCMatrix(const Eigen::Vector3d&) const;
  
  // damping vector of the vehicle
  inline Eigen::Vector3d DampingVector(const Eigen::Vector3d&) const;
  
  
  
public:
    // brief constructor
    BasicController(const VesselControlProperty& vessel, const double& step_size);
    
    // brief a destructor
    ~BasicController(){}
    
    // return control output
    Eigen::Vector3d getOutput() const
    { 
        return output_;
    }
    
    // return time step size of the controller
    double getStepSize() const 
    {
        return step_size_;
    }
    
    // set current position of vessel
    Eigen::Vector3d setVesselPosition(const Eigen::Vector3d& pose)
    {
        return pose;
    }
    
    // set current velocity of vessel
    Eigen::Vector3d setVesselVelocity(const Eigen::Vector3d& vel)
    {
        return vel;
    }
    
    
    // set the position of course
    Eigen::Vector3d setCoursePosition(const Eigen::Vector3d& pose)
    {
        return pose;
    }
    
    
    // set the velocity of course
    Eigen::Vector3d setCourseVelocity(const Eigen::Vector3d& vel)
    {
        return vel;
    }
    
    
    // set the acceleration of course
    Eigen::Vector3d setCourseAcceleration(const Eigen::Vector3d& acc)
    {
        return acc;
    }
  
};


#endif
