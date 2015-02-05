#include "ros_server.h"

using namespace dual_manipulation::ik_control;

ros_server::ros_server()
{
  service = node.advertiseService("ik_ros_service", &ros_server::ik_ros_service, this);
}

bool ros_server::ik_ros_service(dual_manipulation_shared::ik_service::Request &req, dual_manipulation_shared::ik_service::Response &res)
{

    if(IKControl_.perform_ik(req))
    {
	res.ack = true;
	geometry_msgs::Pose ee_pose = req.ee_pose.at(0);
	
	if (req.ee_name != "both_hands")
	{
	  ROS_INFO("Accepted request to perform ik for ee: %s, in %f [s] and pose: {p.x: %f, p.y: %f, p.z: %f, q.x: %f, q.y: %f, q.z: %f, q.w: %f }",
		  req.ee_name.c_str(),req.time,ee_pose.position.x,ee_pose.position.y,ee_pose.position.z,
		  ee_pose.orientation.x,ee_pose.orientation.y,ee_pose.orientation.z,ee_pose.orientation.w);
	}
	else
	{
	  ROS_INFO("Accepted request to perform ik for ee: %s",req.ee_name.c_str());
	}
    }
    else
    {
        ROS_WARN("!! Request to perform ik denied !!");
	res.ack = false;
    }

    return res.ack;
}

ros_server::~ros_server()
{

}
