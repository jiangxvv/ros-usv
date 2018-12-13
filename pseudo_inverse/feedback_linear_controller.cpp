// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class for feedback linear controller
#include<Eigen/Dense>
#include"basic_controller.h"
#include"feedback_linear_controller.h"

FeedbackLinearController::FeedbackLinearController(const VesselControlProperty &vessel, 
                                                   const double &step_size, const double &kp, const double &k)
       :BasicController(vessel, step_size), kp_(kp), k_(k)
{

}




void FeedbackLinearController::ComputeControl()
{
    //the parameters of the equation, M, C, D, here D is a vector.
    Eigen::Matrix3d M=InertiaMatrix();
    Eigen::Matrix3d C=InertiaCCMatrix(vessel_velocity_);
    Eigen::Vector3d D=DampingVector(vessel_velocity_);
//    std::cout<<M<<std::endl;
//    std::cout<<C<<std::endl;
//    std::cout<<D<<std::endl;
    double r=vessel_velocity_[2];// yaw vessel_velocity_
    Eigen::Matrix3d SS;
    SS<<0, -r, 0, r, 0, 0, 0, 0, 0;
    
    
    // rotation matrix
    Eigen::Matrix3d R=BasicController::RotationMatrix(vessel_position_);
    Eigen::Matrix3d Rt=R.transpose();
    
    error_pose_=-Rt*(vessel_position_-course_position_);
    error_vel_=Rt*course_velocity_-vessel_velocity_;
    Eigen::Vector3d error=error_vel_+kp_*error_pose_;
    
    
    output_=C*vessel_velocity_+D+error_pose_+k_*error
            +M*(-SS*(Rt*course_velocity_+kp_*error_pose_)+Rt*course_acceleration_
            +kp_*error_vel_);
                  
}

