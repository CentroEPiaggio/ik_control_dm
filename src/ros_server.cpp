#include "ros_server.h"

using namespace dual_manipulation::ik_control;

ros_server::ros_server()
{
  service = node.advertiseService("ik_ros_service", &ros_server::ik_ros_service, this);
  
  getKinematicModel();

  IKControl_.initializeRobotModel(kinematic_model_);
}

bool ros_server::ik_ros_service(dual_manipulation_shared::ik_service::Request &req, dual_manipulation_shared::ik_service::Response &res)
{

    if(IKControl_.perform_ik(req))
    {
	res.ack = true;
	ROS_INFO("Accepted request to perform ik for ee: %s, in %f [s] and pose: {p.x: %f, p.y: %f, p.z: %f, q.x: %f, q.y: %f, q.z: %f, q.w: %f }",
		  req.ee_name.c_str(),req.time,req.ee_pose.position.x,req.ee_pose.position.y,req.ee_pose.position.z,
		  req.ee_pose.orientation.x,req.ee_pose.orientation.y,req.ee_pose.orientation.z,req.ee_pose.orientation.w);
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

// Get the robot kinematic model from ROS
void ros_server::getKinematicModel()
{
  /* Load the robot model */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  /* Get a shared pointer to the model */
  kinematic_model_ = robot_model_loader.getModel();
  
  return;
}

// void ros_server::getURDF()
// {
//   std::string urdf_string;
// 
//   // search and wait for robot_description on param server
//   while (urdf_string.empty())
//   {
//     std::string search_param_name;
//     if (node.searchParam(param_name, search_param_name))
//     {
//       ROS_INFO_ONCE_NAMED("dual_manipulation::ik_control::ros_server", "dual_manipulation::ik_control::ros_server is waiting for model"
//         " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());
// 
//       node.getParam(search_param_name, urdf_string);
//     }
//     else
//     {
//       ROS_INFO_ONCE_NAMED("dual_manipulation::ik_control::ros_server", "dual_manipulation::ik_control::ros_server is waiting for model"
//         " URDF in parameter %s on the ROS param server.", param_name.c_str());
// 
//       node.getParam(param_name, urdf_string);
//     }
// 
//     usleep(100000);
//   }
//   ROS_DEBUG_STREAM_NAMED("dual_manipulation::ik_control::ros_server", "dual_manipulation::ik_control::ros_server : Recieved urdf from param server, parsing...");
// 
//   if (urdf_model_.initString(urdf_string))
//   {
//     ROS_INFO_ONCE_NAMED("dual_manipulation::ik_control::ros_server", "dual_manipulation::ik_control::ros_server : Robot model loaded.");
//   }
//   else
//   {
//     ROS_ERROR("dual_manipulation::ik_control::ros_server : Unable to load URDF model");
//   }
//   return;
// }
