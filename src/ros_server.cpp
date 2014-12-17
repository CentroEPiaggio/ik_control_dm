#include "ros_server.h"

using namespace dual_manipulation::ik_control;

ros_server::ros_server()
{
    service = node.advertiseService("ik_ros_service", &ros_server::ik_ros_service, this);
}

bool ros_server::ik_ros_service(dual_manipulation_shared::ik_service::Request &req, dual_manipulation_shared::ik_service::Response &res)
{
    res.ack = true;

    return true;
}

ros_server::~ros_server()
{

}