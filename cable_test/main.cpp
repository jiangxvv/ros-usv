#include <iostream>
#include<Eigen/Dense>
#include<fstream>
#include<string>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>


using namespace std;

int main()
{
    std::vector<double> storingforce_;
    std::vector<double> storinglength_;
    std::fstream file;
//    file.open("cable.txt");
    file.open("/home/jiangxvv/catkin_ws/src/gnc_cable/cfg/cable.txt");
    if(file.fail())
    {
        std::cout<< "Cable file reads fail!"<<std::endl;
    }


    // reading matrix data from txt
    std::string str;
    std::vector<std::vector<double>> num;

    while(std::getline(file, str))
    {
        //std::cout<<str<<std::endl;
        std::istringstream input(str);
        std::vector<double> tmp;
        double a;
        while(input>>a)
            tmp.push_back(a);
        num.push_back(tmp);

    }
    cout<<num.size()<<endl;
//    int t=num.size();
    for(int i=0; i<num.size()-1; ++i)
    {
        storingforce_.push_back(num[i][1]);
        storinglength_.push_back(num[i][0]);
        //cout<<storinglength_[i]<<", "<<storingforce_[i]<<endl;

    }

    int data_size=storingforce_.size();
    cout<<data_size<<endl;
//    double x[data_size];
//    double y[data_size];

    double *x;
    double *y;
    x=new double[data_size];
    y=new double[data_size];
    for (size_t i=0; i<data_size; i++)
    {
        x[i]=storinglength_[i];//length
        y[i]=storingforce_[i];//force
        //cout<<x[i]<<", "<<y[i]<<endl;
    }
    cout<<data_size<<endl;

    gsl_interp_accel *acc=
            gsl_interp_accel_alloc();
    gsl_spline *spline
            =gsl_spline_alloc(gsl_interp_cspline, data_size);
    gsl_spline_init(spline, x, y, data_size);

    double f=gsl_spline_eval(spline, 499.5, acc);
    gsl_interp_accel_reset(acc);

    gsl_spline_free(spline);
    gsl_interp_accel_free(acc);
    cout<<f<<endl;

//    delete[] x;
//    delete[] y;


    cout << "Hello World!" << endl;
    return 0;
}


//void readStoringData()
//{
//    std::fstream file;
//    file.open("home/jiangxvv/catkin_ws/src/gnc_cable/cfg/cable.txt");
//    if(file.fail())
//    {
//        std::cout<< "cable file reads fail!"<<std::endl;
//    }


//    // reading matrix data from txt
//    std::string str;
//    std::vector<std::vector<double>> num;

//    while(std::getline(file, str))
//    {
//        std::cout<<str<<std::endl;
//        std::istringstream input(str);
//        std::vector<double> tmp;
//        double a;
//        while(input>>a)
//            tmp.push_back(a);
//        num.push_back(tmp);

//    }

//    for(int i=0; i<num.size(); ++i)
//    {
//        storingforce_.push_back(num[i][0]);
//        storinglength_.push_back(num[i][1]);

//    }

//}
