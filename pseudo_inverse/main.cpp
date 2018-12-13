#include <iostream>
#include<Eigen/Dense>
#include<memory>
#include"variable_thruster_allocation.h"
#include"basic_controller.h"
#include"feedback_linear_controller.h"



//using namespace std;

int main()
{
    Eigen::Vector3d tug1_position;
    Eigen::Vector3d tug2_position;
    tug1_position<<1, 1, 0;
    tug2_position<<1, -1, 0;

    std::unique_ptr<VariableThrusterAllocation> Ptr_allocation;
    std::unique_ptr<FeedbackLinearController> Ptr_controller;

    Ptr_allocation=std::unique_ptr<VariableThrusterAllocation>(new
             VariableThrusterAllocation(tug1_position, tug2_position));

    VesselControlProperty config;
    config.name="mothership";
    config.mass=3.34E7;
    config.radius=33.54;
    config.a11=1.2E7;
    config.a22=3.3E7;
    config.a23=0;
    config.a33=2.8E10;
    config.xg=0;
    config.d11=0.8;
    config.d22=1.7;
    config.d33=0;

    double time_step=0.05;
    double kp=5.0;
    double k=1.0;

    Ptr_controller=std::unique_ptr<FeedbackLinearController>(new
             FeedbackLinearController(config, time_step, kp, k));

    Eigen::Vector3d pose, vel, accel;
    Eigen::Vector3d pd, pd1, pd2;
    pose<<1, 0, 0;
    vel<<0, 0, 0;
    accel<<0, 0, 0;
    pd<<1.1, 0, 0;
    pd1<<0.1, 0, 0;
    pd2<<0, 0, 0;

    Eigen::Vector3d force;
    Ptr_controller->setVesselPosition(pose);
    Ptr_controller->setVesselVelocity(vel);
    Ptr_controller->setCoursePosition(pd);
    Ptr_controller->setCourseVelocity(pd1);
    Ptr_controller->setCourseAcceleration(pd2);

    Ptr_controller->ComputeControl();
    force=Ptr_controller->getOutput();
    std::cout<<"force:"<<std::endl;
    std::cout<<force.transpose()<<std::endl;

    Ptr_allocation->ConfigureMatrix2();
    Ptr_allocation->setForceNet(force);

    Eigen::Vector4d tug_force;
    tug_force=Ptr_allocation->ComputeTug2Force();
    std::cout << "Hello World!" << std::endl;
    std::cout<< tug_force<<std::endl;
    system("pause");
    return 0;
}
