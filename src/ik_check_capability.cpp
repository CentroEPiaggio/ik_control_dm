#include "dual_manipulation_ik_control_ik_check/ik_check_capability.h"
#include <dual_manipulation_shared/parsing_utils.h>

#include <moveit_msgs/GetPositionIK.h>
#include <tf/transform_listener.h>

using namespace dual_manipulation::ik_control;

ikCheckCapability::ikCheckCapability()
{
  // load the robot model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  
  initializeIKCheckCapability(robot_model_loader.getModel());
}

ikCheckCapability::ikCheckCapability(const moveit::core::RobotModelPtr& kinematic_model)
{
  initializeIKCheckCapability(kinematic_model);
}

void ikCheckCapability::initializeIKCheckCapability(const moveit::core::RobotModelPtr& kinematic_model)
{
    kinematic_model_ = kinematic_model;
  
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
    
    kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
    kinematic_state_->setToDefaultValues();
    
    // create a local planning scene
    planning_scene_ = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(kinematic_model_));

    // get all possible group names
    group_names_.clear();
    group_names_ = kinematic_model_->getJointModelGroupNames();

    ik_serviceClient_ = node.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    
    scene_sub_ = node.subscribe("/move_group/monitored_planning_scene",1,&ikCheckCapability::scene_callback,this);
    
    collision_request_.verbose = true;
    
    // apart from the first time, when this is done in the constructor after parameters are obtained from the server
    if(is_initialized_)
    {
      setParameterDependentVariables();
    }
}

void ikCheckCapability::setParameterDependentVariables()
{
  is_initialized_ = true;
  
  for(auto& group:group_names_)
  {
    moveit::core::JointModelGroup* jmg = kinematic_model_->getJointModelGroup(group);
    
    // initialize solver parameters for chains (trees won't have a direct solver - subgroups will be used instead)
    if(jmg->isChain())
    {
      jmg->setDefaultIKTimeout(default_ik_timeout_);
      jmg->setDefaultIKAttempts(default_ik_attempts_);
    }
  }
  for(auto& group:group_map_)
    if(std::find(group_names_.begin(),group_names_.end(),group.second) == group_names_.end())
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
  //TODO: when everything will work smoothly, only use find_group_ik
  if(std::find(chain_names_list_.begin(),chain_names_list_.end(),req.ee_name) == chain_names_list_.end())
  {
    ROS_WARN_STREAM("ikCheckCapability::manage_ik : " << req.ee_name << " is not end-effector of a known chain - trying with find_group_ik");
    std::vector <std::vector <double > > solutions;
    return find_group_ik(req.ee_name,req.ee_pose,solutions);
  }
  
  map_mutex_.lock();
  // get variables from class parameters
  moveit::core::JointModelGroup* jmg = kinematic_model_->getJointModelGroup(group_map_.at(req.ee_name));
  std::string ee_link_name;
  if(jmg->isChain())
  {
    const std::pair <std::string, std::string >& ee_parent_group = jmg->getEndEffectorParentGroup();
    ee_link_name = ee_parent_group.second;
  }
  const std::vector <std::string> active_joints = jmg->getActiveJointModelNames();
  std::vector <double> joint_values;
  kinematic_state_->copyJointGroupPositions(jmg,joint_values);
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

bool ikCheckCapability::find_group_ik(std::string group_name, const std::vector< geometry_msgs::Pose >& ee_poses, std::vector< std::vector< double > >& solutions, const std::vector< double >& initial_guess, bool check_collisions, bool return_approximate_solution, unsigned int attempts, double timeout, const std::map<std::string,std::string>& allowed_collisions)
{
  if(group_map_.count(group_name) == 0)
  {
    ROS_WARN_STREAM("ikCheckCapability::find_group_ik : " << group_name << " is not a known group - returning");
    return false;
  }
  std::vector<std::string> chains;
  if(std::find(tree_names_list_.begin(),tree_names_list_.end(),group_name) != tree_names_list_.end())
  {
    assert(tree_composition_.count(group_name) != 0);
    chains = tree_composition_.at(group_name);
  }
  else
    chains.push_back(group_name);
  if(ee_poses.size() != chains.size())
  {
    ROS_WARN_STREAM("ikCheckCapability::find_group_ik : number of ee_poses specified is " << std::to_string(ee_poses.size()) << " <> needed " << chains.size() << " - returning");
    return false;
  }
  
  kinematic_state_->setToDefaultValues();
  const moveit::core::JointModelGroup* jmg = kinematic_model_->getJointModelGroup(group_map_.at(group_name));
  
  if(!initial_guess.empty())
    if(initial_guess.size() == jmg->getActiveJointModelNames().size())
      kinematic_state_->setJointGroupPositions(jmg,initial_guess);
    else
      ROS_WARN_STREAM("ikCheckCapability::find_group_ik : Initial guess passed as parameter has a wrong dimension : using default position instead");

  solutions.clear();
  solutions.resize(chains.size());
  
  // get default allowed collision matrix and add user-specified entries
  acm_.clear();
  acm_ = planning_scene_->getAllowedCollisionMatrix();
  for(auto& ac:allowed_collisions)
    acm_.setEntry(ac.first,ac.second,true);
  
  // TODO: this loop checks serially for each possible subgroup - implement this better, possibly using recursive calls
  // i.e.: same signature of find_group_ik but with a vector of groups and an index, if the index is last element just do the call with attempts trials, else do a for loop with
  // attempts cycles and in each cycle call 1 attempt of this and the same function with index+1
  for(int i=0; i<chains.size(); i++)
  {
    if(!find_ik(chains.at(i),ee_poses.at(i),solutions.at(i),std::vector<double>(),check_collisions,return_approximate_solution,attempts,timeout))
      return false;
  }
  
  return true;
}

void ikCheckCapability::scene_callback(const moveit_msgs::PlanningScene::ConstPtr& plan_msg)
{
  ROS_INFO_STREAM("ikCheckCapability::scene_callback : updating planning scene");
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

bool ikCheckCapability::find_ik(std::string ee_name, const geometry_msgs::Pose& ee_pose, std::vector< double >& solution, const std::vector< double >& initial_guess, bool check_collisions, bool return_approximate_solution, unsigned int attempts, double timeout)
{
  std::unique_lock<std::mutex>(map_mutex_);
  
  if(!kinematic_model_->hasEndEffector(ee_name))
  {
    ROS_ERROR_STREAM("End-effector " << ee_name << " not found : returning!");
    return true;
  }
  
  // construct necessary inputs
  const moveit::core::JointModelGroup* jmg = kinematic_model_->getEndEffector(ee_name);
  
  // // NOTE: this should be alredy done inside setFromIK anyway...
  // if (attempts == 0)
  //   attempts = default_ik_attempts_;
  // if (timeout == 0.0)
  //   timeout = default_ik_timeout_;
  
  moveit::core::GroupStateValidityCallbackFn constraint;
  if (check_collisions)
  {
    constraint = boost::bind(&ikCheckCapability::is_collision_free, this,_1,_2,_3);
  }
  kinematics::KinematicsQueryOptions options;
  options.return_approximate_solution = return_approximate_solution;
  
  if(!initial_guess.empty())
    if(initial_guess.size() == jmg->getActiveJointModelNames().size())
      kinematic_state_->setJointGroupPositions(jmg,initial_guess);
    else
      ROS_WARN_STREAM("Initial guess passed as parameter has a wrong dimension : using default position instead");
  
  if(!kinematic_state_->setFromIK(jmg,ee_pose,attempts,timeout,constraint,options))
    return false;
  
  kinematic_state_->copyJointGroupPositions(jmg,solution);
  return true;
}

bool ikCheckCapability::find_ik(std::string group_name, std::vector< geometry_msgs::Pose > ee_poses, std::vector<std::vector<double>>& solutions)
{
  return true;
}

bool ikCheckCapability::is_collision_free(moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup *jmg, const double* q)
{
  ROS_INFO_STREAM("ikCheckCapability::is_collision_free has been called for group " << jmg->getName());
  
  collision_result_.clear();
  robot_state->setJointGroupPositions(jmg,q);
  planning_scene_->checkCollision(collision_request_, collision_result_,*robot_state,acm_);

//   std::cout << "Found " << collision_result_.contact_count << " contact(s) (up to a max of " << collision_request_.max_contacts << "):\n";
//   for(auto& coll:collision_result_.contacts)
//     std::cout << coll.first.first << " <> " << coll.first.second << " = " << coll.second.size() << std::endl;

  return (!collision_result_.collision);
}
