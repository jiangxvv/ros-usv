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


#ifndef VARIABLE_THRUSTER_ALLOCATION_H
#define VARIABLE_THRUSTER_ALLOCATION_H

#include<Eigen/Dense>
#include<cmath>

class VariableThrusterAllocation 
{
public:
    // constructor
    VariableThrusterAllocation(const Eigen::Vector3d& tug1_position,
                               const Eigen::Vector3d& tug2_position);
    
    VariableThrusterAllocation(const Eigen::Vector3d& tug1_position,
                               const Eigen::Vector3d& tug2_position,
                               const Eigen::Vector3d& tug3_position,
                               const Eigen::Vector3d& tug4_position);
    
    
    
    
    
    // private:
    
    
protected:
    // mew matrix types
    typedef Eigen::Matrix<double, 4, 1> Vector4d;
    typedef Eigen::Matrix<double, 3, 4> Matrix34d;
    typedef Eigen::Matrix<double, 4, 3> Matrix43d;
    //     typedef Eigen::Matrix<double, 4, 1> Vector4d;
    typedef Eigen::Matrix<double, 3, 8> Matrix38d;
    
        double x1_, y1_, alpha1_; //alpha is du.
        double x2_, y2_, alpha2_;
        double x3_, y3_, alpha3_; //alpha is du.
        double x4_, y4_, alpha4_;

        
        // the relative position of tug with respect to mothership
        Eigen::Vector3d tug1_position_;
        Eigen::Vector3d tug2_position_;
        Eigen::Vector3d tug3_position_;
        Eigen::Vector3d tug4_position_;

        

        Matrix34d B2_;
        Matrix38d B4_;
        
        

        Eigen::Vector4d tug2_force_;
        
        Eigen::Vector3d force1_;
        Eigen::Vector3d force2_;
        Eigen::Vector3d force3_;
        Eigen::Vector3d force4_;
        
        
        
        Eigen::Vector3d force_net_;
        
        void ConfigureMatrix2();
        void ConfigureMatrix4();
        
        
        
        Eigen::Vector4d ComputeTug2Force();
        Eigen::Vector4d ComputeTug4Force();
        
        
        void setForceNet(const Eigen::Vector3d& F)
        {
            force_net_=F;
        }
        
        
            
                
};




#endif
