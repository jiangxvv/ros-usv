// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class to transmit the force to position 

#ifndef CABLE_H
#define CABLE_H

#include<iostream>
#include<Eigen/Dense>
#include<fstream>
#include<string>
#include<memory>
// #include<boost/math/interpolators/barycentric_rational.hpp>


#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>




class Cable 
{
public:
    
    // constructor
    Cable(const double& length);
    
    //destructor
    ~Cable()
    {
        // deallocate array
        delete[] x_;
        delete[] y_;
        
        // delete gsl 1D interpolation interface
        gsl_spline_free(spline_);
        gsl_interp_accel_free(acc_);
        
        gsl_spline_free(spline1_);
        gsl_interp_accel_free(acc1_);
    }
    
    // private field and method
private:
    double length_;
    double force_;
    
    
    // gsl 1D interpolation interface
    double *x_, *y_;
    int data_size_;
    
    std::vector<double> storingforce_;
    std::vector<double> storinglength_;
    
    gsl_interp_accel *acc_;
    gsl_spline *spline_;
    
    gsl_interp_accel *acc1_;
    gsl_spline *spline1_;

public:

    // method reading the relation between force and 
    void readStoringData();
    
    // private method which computes the stretch length through the force 
    double computeLength(const double& force);
    
    // compute force through the length
    double conputeForce(const double& length);
};









#endif