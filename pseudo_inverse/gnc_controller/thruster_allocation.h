// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class for thruster allocation to each tug 

#include <ros/ros.h>
#include<Eigen/Dense>
#include<cmath>
#include<boost/math/constants/constants.hpp>
#include<memory>
#include<cstdlib>

#include"usv_towing/actuation.h"
#include"usv_towing/control.h"



class ThrusterAllocation
{

	// const pi
	const double pi=boost::math::constants::pi<double>();

public:
	// constructor
	ThrusterAllocation(const double& time_step);

	// public method
	int run()
	{
		ros::spin();
		return EXIT_FAILURE;
	}

 	~ThrusterAllocation();

private:
// private field

	// mew matrix types
	typedef Eigen::Matrix<double, 4, 1> Vector4d;
	typedef Eigen::Matrix<double, 3, 4> Matrix34d;

	// configure matrix
	ThrusterAllocation::Matrix34d B_;

	// T_net, T_i
	ThrusterAllocation::Vector4d  Tau_i_;

	Eigen::Vector3d Tau_net_;

	

	// positions and force orientations of four tugs
	double x1_, y1_, alpha1_;
	double x2_, y2_ ,alpha2_;
	double x3_, y3_, alpha3_;
	double x4_, y4_, alpha4_;

	// ros nodehandle
	ros::NodeHandle nh_;

	// timer which controls simulation and the publishing rate
	ros::Timer timer_;

	// publishers and messages
	std::unique_ptr<ros::Publisher> pubPtr_act_;
	usv_towing::actuation msg_act_;

	// subscriber
	std::unique_ptr<ros::Subscriber> subPtr_ctrl_;

	// time step of message publishing
	double step_size_=0.1;

	// flag
	bool received_control_;

// private methods
	void get_configure_params();
	//ThrusterAllocation::Vector4d ComputeOut();
	void timerCallback(const ros::TimerEvent&);
	void callback_sub(const usv_towing::control& msg_ctrl);
};