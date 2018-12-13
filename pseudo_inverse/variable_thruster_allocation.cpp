/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Xu Jiang <jiangxvv@sjtu.edu.cn>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Xu Jiang nor the names of its contributors may
 *     be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */ 

#include<Eigen/Dense>
#include<iostream>
#include"variable_thruster_allocation.h"

VariableThrusterAllocation::VariableThrusterAllocation(const Eigen::Vector3d& tug1_position, const Eigen::Vector3d& tug2_position)
:tug1_position_(tug1_position), tug2_position_(tug2_position)
{
    
}

void VariableThrusterAllocation::ConfigureMatrix2()
{
    x1_=tug1_position_[0]; x2_=tug2_position_[0]; 
    y1_=tug1_position_[1]; y2_=tug2_position_[1]; 
    alpha1_=tug1_position_[2]; alpha2_=tug2_position_[2]; 
    alpha1_=alpha1_/180.0*3.14;
    alpha2_=alpha2_/180.0*3.14;
    
    B2_<<1, 0, 1, 0, 0, 1, 0, 1, -y1_, x1_, -y2_, x2_;
}


Eigen::Vector4d VariableThrusterAllocation::ComputeTug2Force()
{
    ConfigureMatrix2();
    std::cout<<B2_<<std::endl;
    Matrix43d B2_inverse;
    B2_inverse=B2_.transpose()*((B2_*B2_.transpose()).inverse());
    std::cout<<B2_inverse<<std::endl;
    Eigen::Vector4d tug2_force;
    tug2_force=B2_inverse*force_net_;

    std::cout<<tug2_force<<std::endl;
    
    double fx1=tug2_force[0];
    double fy1=tug2_force[1];
    double fx2=tug2_force[2];
    double fy2=tug2_force[3];
    
    double alpha1=atan2(fy1, fx1);
    double alpha2=atan2(fy2, fx2);

    alpha1=alpha1/3.14*180;
    alpha2=alpha2/3.14*180;
    
    double f1=sqrt(pow(fx1, 2)+pow(fy1, 2));
    double f2=sqrt(pow(fx2, 2)+pow(fy2, 2));
    
    tug2_force<<f1, alpha1, f2, alpha2;

    return tug2_force;
}

