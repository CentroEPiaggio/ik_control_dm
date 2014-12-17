#include "ros_server.h"
#include <iostream>

using namespace dual_manipulation::ik_control;

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> IK CONTROL running"<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "dual_manipulation_ik_control");

    ros_server server;

    ros::spin();

    return 0;
}