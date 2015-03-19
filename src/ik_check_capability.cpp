#include "ik_check_capability.h"
#include <dual_manipulation_shared/parsing_utils.h>
#include <moveit_msgs/GetPositionIK.h>

using namespace dual_manipulation::ik_control;

ikCheckCapability::ikCheckCapability()
{
    setDefaultParameters();

    if (node.getParam("ik_control_parameters", ik_control_params))
      parseParameters(ik_control_params);
    
    for(auto group_name:group_map_)
      moveGroups_[group_name.first] = new move_group_interface::MoveGroup( group_name.second, boost::shared_ptr<tf::Transformer>(), ros::Duration(5, 0) );
    
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
    
    for(auto group:moveGroups_)
	delete group.second;
}

void ikCheckCapability::setDefaultParameters()
{
    chain_names_list_.clear();
    chain_names_list_.assign({"left_hand","right_hand"});
    tree_names_list_.clear();
    tree_names_list_.assign({"both_hands"});
    
    group_map_.clear();
    group_map_["left_hand"] = "left_hand_arm";
    group_map_["right_hand"] = "right_hand_arm";
    group_map_["both_hands"] = "dual_hand_arm";
    
    // apart from the first time, when this is done in the constructor after parameters are obtained from the server
    if(moveGroups_.size() > 0)
    {
      for(auto group:moveGroups_)
	delete group.second;
      moveGroups_.clear();
      
      for(auto group_name:group_map_)
	moveGroups_[group_name.first] = new move_group_interface::MoveGroup( group_name.second, boost::shared_ptr<tf::Transformer>(), ros::Duration(5, 0) );
    }
}

void ikCheckCapability::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    std::vector<std::string> names_list;
    parseSingleParameter(params,names_list,"chain_group_names");
    if(names_list.size() > 0)
    {
      chain_names_list_.swap(names_list);
      names_list.clear();
    }
    parseSingleParameter(params,names_list,"tree_group_names");
    if(names_list.size() > 0)
    {
      tree_names_list_.swap(names_list);
      names_list.clear();
    }
    
    std::map<std::string,std::string> map_tmp;
    parseSingleParameter(params,map_tmp,"group_map",chain_names_list_);
    parseSingleParameter(params,map_tmp,"group_map",tree_names_list_);
    if(map_tmp.size() > 0)
    {
      group_map_.swap(map_tmp);
      map_tmp.clear();
    }
}

bool ikCheckCapability::manage_ik(dual_manipulation_shared::ik_service::Request req)
{
  if(std::find(chain_names_list_.begin(),chain_names_list_.end(),req.ee_name) == chain_names_list_.end())
  {
    ROS_WARN_STREAM("ikCheckCapability::manage_ik : " << req.ee_name << " is not end-effector of a known chain - returning");
    return false;
  }
  
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
	ROS_INFO_STREAM("ikCheckCapability::manage_ik : error_code.val = " << service_response.error_code.val << std::endl);
	break;
    }
    else
	ROS_WARN_STREAM("ikCheckCapability::manage_ik : error_code.val = " << service_response.error_code.val << std::endl);
  }
  
  // for (auto item:service_response.solution.joint_state.position)
  //   std::cout << item << " | ";
  // std::cout << std::endl;
  
  return (service_response.error_code.val == 1);
}
