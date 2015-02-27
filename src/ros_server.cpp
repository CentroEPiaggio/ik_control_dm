#include "ros_server.h"

using namespace dual_manipulation::ik_control;

ros_server::ros_server()
{
  service_ik = node.advertiseService("ik_ros_service", &ros_server::ik_ros_service, this);
  service_scene_object = node.advertiseService("scene_object_ros_service", &ros_server::scene_object_ros_service, this);
}

bool ros_server::ik_ros_service(dual_manipulation_shared::ik_service::Request &req, dual_manipulation_shared::ik_service::Response &res)
{

    if(IKControl_.perform_ik(req))
    {
	res.ack = true;
	ROS_INFO_STREAM("ros_server::ik_ros_service : Accepted request to perform " << req.command << " command for end-effector " << req.ee_name);
    }
    else
    {
        ROS_WARN_STREAM("!! ros_server::ik_ros_service : Request to perform " << req.command << " command for end-effector " << req.ee_name << " DENIED !!");
	res.ack = false;
    }

    return res.ack;
}

bool ros_server::scene_object_ros_service(dual_manipulation_shared::scene_object_service::Request& req, dual_manipulation_shared::scene_object_service::Response& res)
{
    if(IKControl_.manage_object(req))
    {
	res.ack = true;
	ROS_INFO("Accepted request to manage (%s) the object %s",req.command.c_str(),req.attObject.object.id.c_str());
    }
    else
    {
        ROS_WARN("!! Request to manage (%s) the object %s DENIED !!",req.command.c_str(),req.attObject.object.id.c_str());
	res.ack = false;
    }

    return res.ack;
}

ros_server::~ros_server()
{

}
