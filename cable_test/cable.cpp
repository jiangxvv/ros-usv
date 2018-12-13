// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class to transmit the force to position

#include"cable.h"

#include<iostream>
#include<Eigen/Dense>
#include<fstream>
#include<sstream>
#include<string>
#include<memory>

#include<gsl/gsl_errno.h>
#include<gsl/gsl_spline.h>

Cable::Cable(const double &length) :length_(length) 
{

    readStoringData();
    data_size_=storingforce_.size();
    std::cout<<data_size_<<std::endl;
    
    acc_=gsl_interp_accel_alloc();
    spline_=gsl_spline_alloc(gsl_interp_steffen, data_size_);
    
    acc1_=gsl_interp_accel_alloc();
    spline1_=gsl_spline_alloc(gsl_interp_steffen, data_size_);
    
    x_=new double[data_size_];
    y_=new double[data_size_];
    
    for (size_t i=0; i<data_size_; i++)
    {
        x_[i]=storinglength_[i];//length
        y_[i]=storingforce_[i];//force
    }
    
}


void Cable::readStoringData()
{
    std::fstream file;
    file.open("/home/jiangxvv/catkin_ws/src/gnc_cable/cfg/cable.txt");
    if(file.fail())
    {
        std::cout<< "cable file reads fail!"<<std::endl;
    }      
    
    // reading matrix data from txt
    std::string str;
    std::vector<std::vector<double>> num;
    
    while(std::getline(file, str))
    {
//         std::cout<<str<<std::endl;
        std::istringstream input(str);
        std::vector<double> tmp;
        double a;
        while(input>>a)
            tmp.push_back(a);
        num.push_back(tmp);   
    }
    
    for(int i=0; i<num.size()-1; ++i)
    {
        storingforce_.push_back(num[i][1]);
        storinglength_.push_back(num[i][0]);
        
    }
    //std::cout<<storingforce_[1]<<std::endl;
}



//transfer the force to length
double Cable::computeLength(const double &force)
{
//         acc_=gsl_interp_accel_alloc();
//         spline_=gsl_spline_alloc(gsl_interp_steffen, data_size_);
        
    gsl_spline_init(spline_, y_, x_, data_size_);
    double length= gsl_spline_eval(spline_, force, acc_);     
    return length;
}




// transfer the length to force
double Cable::conputeForce(const double &length)
{
//         acc1_=gsl_interp_accel_alloc();
//         spline1_=gsl_spline_alloc(gsl_interp_steffen, data_size_);
        
    gsl_spline_init(spline1_, x_, y_, data_size_);
        
    double force=gsl_spline_eval(spline1_, length, acc1_);
        
    return force;
}




// double Cable::computeLength(const double& force)
// {
//     double length=0;
// }
// double Cable::computeLength(const double &force)
// {
//     
//     //boost::math::barycentric_rational<double> interpolator(
//        // storingforce_.data(), storinglength_.data(), storingforce_.size());
//     
//     unsigned int data_size=storingforce_.size();
//     
//     acc_=gsl_interp_accel_alloc();
//     spline_=gsl_spline_alloc(gsl_interp_cspline,data_size);
//     
//     gsl_spline_init(spline_, storingforce_, storinglength_, data_size);
//     
//     double length= gsl_spline_eval(spline_, force, acc_);
//     
//     gsl_spline_free(spline_);
//     gsl_interp_accel_free(acc_);
//     
//     return length;
// }
// 
// double Cable::conputeForce(const double &length)
// {
//     unsigned int data_size=storinglength_.size();
//     
//     acc1_=gsl_interp_accel_alloc();
//     spline1_=gsl_spline_alloc(gsl_interp_cspline, data_size);
//     
//     gsl_spline_init(spline1_, storinglength_, storingforce_, data_size);
//     
//     double force=gsl_spline_eval(spline1_, length, acc1_);
//     
//     gsl_spline_free(spline1_);
//     gsl_interp_accel_free(acc1_);
// }



























