#include "ros_server.h"

using namespace dual_manipulation::ik_control;

ros_server::ros_server()
{
    service = node.advertiseService("ik_ros_service", &ros_server::ik_ros_service, this);
}

bool ros_server::ik_ros_service(dual_manipulation_shared::ik_service::Request &req, dual_manipulation_shared::ik_service::Response &res)
{

	res.ack = true;
	ROS_INFO("Accepted request to perform ik for ee: %s, in %f [s] and pose: {p.x: %f, p.y: %f, p.z: %f, q.x: %f, q.y: %f, q.z: %f, q.w: %f }",
		  req.ee_name.c_str(),req.time,req.ee_pose.position.x,req.ee_pose.position.y,req.ee_pose.position.z,
		  req.ee_pose.orientation.x,req.ee_pose.orientation.y,req.ee_pose.orientation.z,req.ee_pose.orientation.w);

    return res.ack;
}

ros_server::~ros_server()
{

}