#include "ik_check_capability.h"
#include <moveit_msgs/GetPositionIK.h>

using namespace dual_manipulation::ik_control;

ikCheckCapability::ikCheckCapability()
{
    group_map_["left_hand"] = "left_hand_arm";
    group_map_["right_hand"] = "right_hand_arm";
    group_map_["both_hands"] = "dual_hand_arm";
    
    moveGroups_["left_hand"] = new move_group_interface::MoveGroup(group_map_.at("left_hand"));
    moveGroups_["right_hand"] = new move_group_interface::MoveGroup(group_map_.at("right_hand"));
    moveGroups_["both_hands"] = new move_group_interface::MoveGroup(group_map_.at("both_hands"));
    
//     kinematics_plugin_["left_hand"] = new kdl_kinematics_plugin::KDLKinematicsPlugin();
//     kinematics_plugin_["right_hand"] = new kdl_kinematics_plugin::KDLKinematicsPlugin();
    
//     // NOTE: attempted value of search_discretization: it's not clear what it is used for
//     kinematics_plugin_.at("left_hand")->initialize("robot_description",group_map_.at("left_hand"),"world",moveGroups_.at("left_hand")->getEndEffectorLink(),0.005);
//     kinematics_plugin_.at("right_hand")->initialize("robot_description",group_map_.at("right_hand"),"world",moveGroups_.at("right_hand")->getEndEffectorLink(),0.005);
    
//     kinematics::KinematicsQueryOptions opt;
//     opt.return_approximate_solution = true;
//     kinematics_plugin_.at("left_hand")->getPositionIK();
    
    ik_serviceClient_ = node.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
}

ikCheckCapability::~ikCheckCapability()
{
//     delete kinematics_plugin_.at("left_hand");
//     delete kinematics_plugin_.at("right_hand");
    
    delete moveGroups_.at("left_hand");
    delete moveGroups_.at("right_hand");
    delete moveGroups_.at("both_hands");
}

bool ikCheckCapability::manage_ik(dual_manipulation_shared::ik_service::Request req)
{
  moveit_msgs::GetPositionIK::Request service_request;
  moveit_msgs::GetPositionIK::Response service_response; 
  service_request.ik_request.group_name = group_map_.at(req.ee_name);
  service_request.ik_request.pose_stamped.header.frame_id = "world";  
  service_request.ik_request.pose_stamped.pose = req.ee_pose.at(0);
  service_request.ik_request.ik_link_name = moveGroups_.at(req.ee_name)->getEndEffectorLink();
  service_request.ik_request.robot_state.joint_state.name = moveGroups_.at(req.ee_name)->getActiveJoints();
  service_request.ik_request.avoid_collisions = true;
  service_request.ik_request.timeout = ros::Duration(0.02);
  service_request.ik_request.attempts = 1;
  
  for(int i=0; i<10; i++)
  {
    service_request.ik_request.robot_state.joint_state.position = moveGroups_.at(req.ee_name)->getCurrentJointValues();
    ik_serviceClient_.call(service_request, service_response);
  
    if (service_response.error_code.val == 1)
    {
	// did it!
	ROS_INFO_STREAM("IKControl::ik_check_thread: error_code.val = " << service_response.error_code.val << std::endl);
	break;
    }
    else
	ROS_WARN_STREAM("IKControl::ik_check_thread: error_code.val = " << service_response.error_code.val << std::endl);
  }
  
  // for (auto item:service_response.solution.joint_state.position)
  //   std::cout << item << " | ";
  // std::cout << std::endl;
  
  return (service_response.error_code.val == 1);
}
