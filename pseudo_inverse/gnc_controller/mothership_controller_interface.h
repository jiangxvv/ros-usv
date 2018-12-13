// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class of the central controller

#ifndef MOTHERSHIP_CONTROLLER_INTERFACE_H
#define MOTHERSHIP_CONTROLLER_INTERFACE_H

#include<ros/ros.h>
#include <memory>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Dense>
#include "gnc_msgs/control.h"
#include"gnc_msgs/course.h"
#include"feedback_linear_controller.h"
#include"basic_controller.h"



// class define
class MotherShipControllerInterface
{
        
public:
// constructor
        MotherShipControllerInterface(const VesselControlProperty& vessel,
                                      double& time_step,const double& kp, 
                                      const double& k);

        // public methods
        int run()
        {
                ros::spin();
                return EXIT_FAILURE;
        }
        
        virtual ~MotherShipControllerInterface();

private:
        // private field
        // ros node
        ros::NodeHandle nh_;

        // publisher and message of the actuation signal
        std::unique_ptr<ros::Publisher> pubPtr_actuation_;
        gnc_msgs::control msg_actuation_;

        // create a timer to control the publishing of control commands
        ros::Timer timer_;

        // time step 
        double time_step_;
        
        VesselControlProperty vessel_;

        // subscribers
        std::unique_ptr<ros::Subscriber> sub_pos_;
        std::unique_ptr<ros::Subscriber> sub_vel_;
        std::unique_ptr<ros::Subscriber> sub_course_;
        
        std::unique_ptr<FeedbackLinearController> Ptr_controller_;

//         // mothership class
//         std::unique_ptr<MotherShip> Ptr_vehicle_;
        
        // actuation computed by the controller
        Eigen::Vector3d actuation_;

        // helper variable
        bool received_course_info_=false;
        
        // parameters of controller
        double kp_, k_;

       

                

        void timerCallback(const ros::TimerEvent&);
        
        void  callback_pose(const geometry_msgs::Pose2D& msg_pose);
        void callback_vel(const geometry_msgs::Twist& msg_vel);
        void callback_course(const usv_towing::course& msg_course);
//         void get_ros_param();

public:
        Eigen::Vector3d ActuationCompute();

};

#endif