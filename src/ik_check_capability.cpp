#include "ik_check_capability.h"
#include <dual_manipulation_shared/parsing_utils.h>

#include <moveit_msgs/GetPositionIK.h>
#include <tf/transform_listener.h>

using namespace dual_manipulation::ik_control;

ikCheckCapability::ikCheckCapability()
{
    setDefaultParameters();

    if (node.getParam("ik_control_parameters", ik_control_params))
      parseParameters(ik_control_params);
    
    setParameterDependentVariables();
}

ikCheckCapability::~ikCheckCapability()
{
}

void ikCheckCapability::setDefaultParameters()
{
    chain_names_list_.clear();
    chain_names_list_.assign({"left_hand","right_hand"});
    tree_names_list_.clear();
    tree_names_list_.assign({"both_hands"});
    tree_composition_.clear();
    tree_composition_["both_hands"] = std::vector<std::string>({"left_hand","right_hand"});
    
    group_map_.clear();
    group_map_["left_hand"] = "left_hand_arm";
    group_map_["right_hand"] = "right_hand_arm";
    group_map_["both_hands"] = "dual_hand_arm";
    
    // load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model_ = robot_model_loader.getModel();
    kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
    kinematic_state_->setToDefaultValues();
    
    // create a local planning scene
    planning_scene_ = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(kinematic_model_));

    // get all possible group names
    group_names_.clear();
    group_names_ = kinematic_model_->getJointModelGroupNames();

    ik_serviceClient_ = node.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    
    scene_sub_ = node.subscribe("/move_group/monitored_planning_scene",1,&ikCheckCapability::scene_callback,this);
    
    // apart from the first time, when this is done in the constructor after parameters are obtained from the server
    if(is_initialized_)
    {
      jm_groups_.clear();
      
      setParameterDependentVariables();
    }
}

void ikCheckCapability::setParameterDependentVariables()
{
  is_initialized_ = true;
  
  for(auto& group:group_names_)
  {
    jm_groups_[group] = kinematic_model_->getJointModelGroup(group);
    
    // initialize solver parameters for chains (trees won't have a direct solver - subgroups will be used instead)
    if(jm_groups_[group]->isChain())
    {
      jm_groups_[group]->setDefaultIKTimeout(default_ik_timeout_);
      jm_groups_[group]->setDefaultIKAttempts(default_ik_attempts_);
    }
  }
  for(auto& group:group_map_)
    if(jm_groups_.count(group.second) == 0)
      ROS_ERROR_STREAM("Specified group \"" << group.second << "\" (named : " << group.first << ") not present : IK check will not be possible for that group!!!");
}

void ikCheckCapability::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    parseSingleParameter(params,chain_names_list_,"chain_group_names",1);
    parseSingleParameter(params,tree_names_list_,"tree_group_names",1);
    
    // list of chains composing each tree
    if(params.hasMember("tree_composition"))
    {
      std::map<std::string,std::vector<std::string>> tc_tmp;
      for(auto tree:tree_names_list_)
      {
	parseSingleParameter(params["tree_composition"],tc_tmp[tree],tree);
	if(tc_tmp.at(tree).empty())
	  tc_tmp.erase(tree);
      }
      if(!tc_tmp.empty())
      {
	tree_composition_.swap(tc_tmp);
	tc_tmp.clear();
      }
    }
    for(auto tree:tree_names_list_)
      if(!tree_composition_.count(tree) || tree_composition_.at(tree).size() == 0)
	ROS_WARN_STREAM("No composition is specified for tree '" << tree << "': check the yaml configuration.");
    
    std::map<std::string,std::string> map_tmp,map_tmp_tree;
    parseSingleParameter(params,map_tmp,"group_map",chain_names_list_);
    parseSingleParameter(params,map_tmp_tree,"group_map",tree_names_list_);
    if(!map_tmp_tree.empty())
      for(auto tree:map_tmp_tree)
	map_tmp[tree.first] = tree.second;
    if(!map_tmp.empty())
    {
      group_map_.swap(map_tmp);
      map_tmp.clear();
    }
    
    parseSingleParameter(params,default_ik_timeout_,"default_ik_timeout");
    int tmp_int_param = (int)default_ik_attempts_;
    parseSingleParameter(params,tmp_int_param,"default_ik_attempts");
    if(tmp_int_param >= 0)
      default_ik_attempts_ = (unsigned int)tmp_int_param;
    else
      ROS_WARN_STREAM("Attempted to set default_ik_attempts to a negative value; using default instead.");
}

bool ikCheckCapability::manage_ik(dual_manipulation_shared::ik_service::Request req)
{
  if(std::find(chain_names_list_.begin(),chain_names_list_.end(),req.ee_name) == chain_names_list_.end())
  {
    ROS_WARN_STREAM("ikCheckCapability::manage_ik : " << req.ee_name << " is not end-effector of a known chain - returning");
    return false;
  }
  
  map_mutex_.lock();
  // get variables from class parameters
  std::string ee_link_name;
  if(jm_groups_.at(group_map_.at(req.ee_name))->isChain())
  {
    const std::pair <std::string, std::string >& ee_parent_group = jm_groups_.at(group_map_.at(req.ee_name))->getEndEffectorParentGroup();
    ee_link_name = ee_parent_group.second;
  }
  const std::vector <std::string> active_joints = jm_groups_.at(group_map_.at(req.ee_name))->getActiveJointModelNames();
  std::vector <double> joint_values;
  kinematic_state_->copyJointGroupPositions(jm_groups_.at(group_map_.at(req.ee_name)),joint_values);
  map_mutex_.unlock();
  
  moveit_msgs::GetPositionIK::Request service_request;
  moveit_msgs::GetPositionIK::Response service_response;
  service_request.ik_request.group_name = group_map_.at(req.ee_name);
  service_request.ik_request.pose_stamped.header.frame_id = "world";
  service_request.ik_request.pose_stamped.pose = req.ee_pose.at(0);
  service_request.ik_request.ik_link_name = ee_link_name;
  service_request.ik_request.robot_state.joint_state.name = active_joints;
  service_request.ik_request.avoid_collisions = true;
  service_request.ik_request.timeout = ros::Duration(0.02);
  service_request.ik_request.attempts = 1;
  
  for(int i=0; i<10; i++)
  {
    service_request.ik_request.robot_state.joint_state.position = joint_values;
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

void ikCheckCapability::scene_callback(const moveit_msgs::PlanningScene::ConstPtr& plan_msg)
{
  // update the internal planning scene, considering whether or not is_diff flag is set to true
  scene_mutex_.lock();
  planning_scene_->usePlanningSceneMsg(*plan_msg);
  scene_mutex_.unlock();
  
  // ROS_INFO_STREAM("ikCheckCapability::scene_callback - plan_msg:\n" << *plan_msg << std::endl);
}

//NOTE: this is a joint model group we could use (all possible groups exist this way, e.g. even head in vito)
// moveit::core::JointModelGroup* bim_group = kinematic_model_->getJointModelGroup("dual_hand_arm");
//NOTE: the following names are from the SRDF file, not the link names
// const std::vector <std::string > ee_names = bim_group->getAttachedEndEffectorNames();
//NOTE: getEndEffectorTips() works if the jm_group is a tree, returning the link names of all the end-effectors
//      BUT does not work for chains (i.e.: works for bimanual group, but doesn't for single hand/arm groups)
// std::vector <std::string > tips;
// bim_group->getEndEffectorTips(tips);
//NOTE: this returns {left_arm | left_hand_arm | right_arm | right_hand_arm}
// const std::vector <std::string >& subgroups = bim_group->getSubgroupNames();
//NOTE: isChain() and isEndEffector() always return false for trees
//NOTE: kinematic_model_->getRootLinkName() could be used instead of "world", just to be sure

bool ikCheckCapability::find_ik(std::string ee_link, geometry_msgs::Pose ee_pose, std::vector<double>& solution)
{
  return true;
}

bool ikCheckCapability::find_ik(std::string group_name, std::vector< geometry_msgs::Pose > ee_poses, std::vector<std::vector<double>>& solutions)
{
  return true;
}

bool ikCheckCapability::is_collision_free()
{
  return true;
}
