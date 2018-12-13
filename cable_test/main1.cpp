
#include"cable.h"
// #include"gnc_cable/cable.h"

int main()
{
//    ros::init(argc, argv, "cable_test_node");
//    ros::NodeHandle nh;


    double length=400;
    Cable cable(length);
    double force=cable.conputeForce(400);
    double l=cable.computeLength(272);
    std::cout<<force<<std::endl;
    std::cout<<l<<std::endl;

    //ros::spin();
    return 0;
}
