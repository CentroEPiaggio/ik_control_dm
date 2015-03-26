#include "ik_control.h"
#include "trajectory_utils.h"
#include <dual_manipulation_shared/parsing_utils.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/String.h>

#define SIMPLE_GRASP 1
#define IK_CHECK_CAPABILITY "ik_check"
#define PLAN_CAPABILITY "plan"
#define MOVE_CAPABILITY "execute"
#define GRASP_CAPABILITY "grasp"
#define UNGRASP_CAPABILITY "ungrasp"
#define HOME_CAPABILITY "home"

using namespace dual_manipulation::ik_control;

ikControl::ikControl()
{
    setDefaultParameters();
    
    if (node.getParam("ik_control_parameters", ik_control_params))
      parseParameters(ik_control_params);
    
    setParameterDependentVariables();
}

void ikControl::setDefaultParameters()
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
    
    position_threshold = 0.0007;
    velocity_threshold = 0.0007;
    hand_max_velocity = 2.0;
    hand_position_threshold = 1.0/200.0;
    
    kinematics_only_ = false;
    
    allowed_collision_prefixes_.clear();
    allowed_collision_prefixes_["left_hand"] = std::vector<std::string>({"left_hand","left_arm_7_link"});
    allowed_collision_prefixes_["right_hand"] = std::vector<std::string>({"right_hand","right_arm_7_link"});
    
    // planner parameters
    planner_id_ = "RRTstarkConfigDefault";
    planning_time_ = 1.0;
    goal_position_tolerance_ = 0.005;
    goal_orientation_tolerance_ = 0.005;
    goal_joint_tolerance_ = 0.005;
    ws_bounds_.assign({-1.2,-1.5,0.1,0.2,1.5,1.5});
    
    // all possible capabilities are defined here, along with the sub-topic they will refer to
    // NOTE: DO NOT CHANGE THIS unless necessary, this are the names known to the outside
    capabilities_.clear();
    capabilities_[MOVE_CAPABILITY] = "action_done";
    capabilities_[PLAN_CAPABILITY] = "planning_done";
    capabilities_[IK_CHECK_CAPABILITY] = "check_done";
    capabilities_[GRASP_CAPABILITY] = "grasp_done";
    capabilities_[UNGRASP_CAPABILITY] = "grasp_done";
    capabilities_[HOME_CAPABILITY] = "action_done";
    // TODO: change topic once home is implemented as a target to be set
    //capabilities_[SET_TARGET_CAPABILITY] = "target_set";
    //capabilities_[HOME_CAPABILITY] = "target_set";
    
    traj_pub_topics_.clear();
    traj_pub_topics_["left_hand"] = "/left_arm/joint_trajectory_controller/command";
    traj_pub_topics_["right_hand"] = "/right_arm/joint_trajectory_controller/command";
    
    hand_synergy_pub_topics_.clear();
    hand_synergy_pub_topics_["left_hand"] = "/left_hand/joint_trajectory_controller/command";
    hand_synergy_pub_topics_["right_hand"] = "/right_hand/joint_trajectory_controller/command";
    
    controller_map_.clear();
    controller_map_["left_hand"] = "/left_arm/joint_trajectory_controller/follow_joint_trajectory/";
    controller_map_["right_hand"] = "/right_arm/joint_trajectory_controller/follow_joint_trajectory/";
    
    hand_actuated_joint_.clear();
    hand_actuated_joint_["left_hand"] = "left_hand_synergy_joint";
    hand_actuated_joint_["right_hand"] = "right_hand_synergy_joint";

    // apart from the first time, when this is done in the constructor after parameters are obtained from the server
    if(moveGroups_.size() > 0)
    {
      for(auto group:moveGroups_)
	delete group.second;
      moveGroups_.clear();
      movePlans_.clear();
      busy.clear();
      hand_pub.clear();
      traj_pub_.clear();
      hand_synergy_pub_.clear();
      
      setParameterDependentVariables();
    }
}

void ikControl::setParameterDependentVariables()
{
  for(auto group_name:group_map_)
  {
    moveGroups_[group_name.first] = new move_group_interface::MoveGroup( group_name.second, boost::shared_ptr<tf::Transformer>(), ros::Duration(5, 0) );
    movePlans_[group_name.first];

    for(auto capability:capabilities_)
    {
      hand_pub[capability.first][group_name.first] = node.advertise<std_msgs::String>("/ik_control/" + group_name.first + "/" + capability.second,1,this);
      busy[capability.first][group_name.first] = false;
    }
  }
  
  for(auto item:moveGroups_)
  {
    item.second->setPlannerId(planner_id_);
    item.second->setPlanningTime(planning_time_);
    item.second->setGoalPositionTolerance(goal_position_tolerance_);
    item.second->setGoalOrientationTolerance(goal_orientation_tolerance_);
    item.second->setGoalJointTolerance(goal_joint_tolerance_);
    item.second->setWorkspace(ws_bounds_.at(0),ws_bounds_.at(1),ws_bounds_.at(2),ws_bounds_.at(3),ws_bounds_.at(4),ws_bounds_.at(5));
  }
  
  for(auto chain_name:chain_names_list_)
  {
    // allowed touch links
    std::vector<std::string> links = moveGroups_.at(chain_name)->getCurrentState()->getRobotModel()->getLinkModelNamesWithCollisionGeometry();
    for (auto link:links)
      for (auto acpref:allowed_collision_prefixes_.at(chain_name))
	if(link.compare(0,acpref.size(),acpref.c_str()) == 0)
	{
	  allowed_collisions_[chain_name].push_back(link);
	  break;
	}

    // JointTrajectory publishers
    traj_pub_[chain_name] = node.advertise<trajectory_msgs::JointTrajectory>(traj_pub_topics_.at(chain_name),1,this);
    hand_synergy_pub_[chain_name] = node.advertise<trajectory_msgs::JointTrajectory>(hand_synergy_pub_topics_.at(chain_name),1,this);
  }
}

void ikControl::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    parseSingleParameter(params,position_threshold,"position_threshold");
    parseSingleParameter(params,velocity_threshold,"velocity_threshold");
    parseSingleParameter(params,hand_max_velocity,"hand_max_velocity");
    parseSingleParameter(params,hand_position_threshold,"hand_position_threshold");
    parseSingleParameter(params,kinematics_only_,"kinematics_only");

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
    
    // allowed collision parameters
    if(params.hasMember("allowed_collision_prefixes"))
    {
      std::map<std::string,std::vector<std::string>> acp_tmp;
      for(auto chain:chain_names_list_)
      {
	parseSingleParameter(params["allowed_collision_prefixes"],acp_tmp[chain],chain);
	if(acp_tmp.at(chain).empty())
	  acp_tmp.erase(chain);
      }
      if(!acp_tmp.empty())
      {
	allowed_collision_prefixes_.swap(acp_tmp);
	acp_tmp.clear();
      }
    }
    
    parseSingleParameter(params,traj_pub_topics_,"traj_pub_topics",chain_names_list_);
    parseSingleParameter(params,hand_synergy_pub_topics_,"hand_synergy_pub_topics",chain_names_list_);
    parseSingleParameter(params,controller_map_,"controller_map",chain_names_list_);
    parseSingleParameter(params,hand_actuated_joint_,"hand_actuated_joint",chain_names_list_);

    // planner parameters
    if(params.hasMember("motion_planner"))
    {
      parseSingleParameter(params["motion_planner"],planner_id_,"planner_id");
      parseSingleParameter(params["motion_planner"],planning_time_,"planning_time");
      parseSingleParameter(params["motion_planner"],goal_position_tolerance_,"goal_position_tolerance");
      parseSingleParameter(params["motion_planner"],goal_orientation_tolerance_,"goal_orientation_tolerance");
      parseSingleParameter(params["motion_planner"],goal_joint_tolerance_,"goal_joint_tolerance");
      parseSingleParameter(params["motion_planner"],ws_bounds_,"workspace_bounds",6);
    }
}

bool ikControl::manage_object(dual_manipulation_shared::scene_object_service::Request& req)
{
    return scene_object_manager_.manage_object(req);
}

bool ikControl::waitForHandMoved(std::string& hand, double hand_target)
{
  ROS_INFO_STREAM("ikControl::waitForHandMoved : entered");
  
  if(kinematics_only_)
  {
    ROS_INFO_STREAM("ikControl::waitForHandMoved : kinematics_only execution - moving on after sleeping 1 second");
    sleep(1);
    return true;
  }

  int counter = 0;
  int hand_index = 0;
  bool good_stop = false;
  sensor_msgs::JointStateConstPtr joint_states;
  
  joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",node,ros::Duration(3));
  for(auto joint:joint_states->name)
  {
    if(joint == hand_actuated_joint_.at(hand))
    {
      break;
    }
    hand_index++;
  }
  if(hand_index >= joint_states->name.size())
  {
    ROS_ERROR_STREAM("ikControl::waitForHandMoved : " << hand_actuated_joint_.at(hand) << " NOT found in /joint_states - returning");
    return false;
  }

  // wait for up to 10 more seconds
  while(counter<100)
  {
    //get joint states
    joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",node,ros::Duration(3));
    
    if(joint_states->name.at(hand_index) != hand_actuated_joint_.at(hand))
    {
      ROS_ERROR("ikControl::waitForHandMoved : joints in joint_states changed order");
      return false;
    }
    
    if (std::norm(joint_states->position.at(hand_index) - hand_target) < hand_position_threshold)
    {
      good_stop = true;
      break;
    }
    usleep(100000);
    counter++;
  }
  
  if(good_stop)
    ROS_INFO("ikControl::waitForHandMoved : exiting with good_stop OK");
  else
    ROS_WARN("ikControl::waitForHandMoved : exiting with error");
  return good_stop;
}

bool ikControl::waitForExecution(std::string ee_name, moveit_msgs::RobotTrajectory traj)
{
  ROS_INFO_STREAM("ikControl::waitForExecution : entered");
  
  if (traj.joint_trajectory.points.size() == 0)
  {
    ROS_WARN_STREAM("ikControl::waitForExecution : the trajectory to wait for was empty");
    return true;
  }
  
  if(kinematics_only_)
  {
    ROS_INFO_STREAM("ikControl::waitForExecution : kinematics_only execution - moving on after sleeping 1 second");
    sleep(1);
    return true;
  }

  map_mutex_.lock();
  int has_ctrl = controller_map_.count(ee_name);
  std::string controller_name;
  if(has_ctrl != 0)
    controller_name = controller_map_.at(ee_name);
  map_mutex_.unlock();
  
  control_msgs::FollowJointTrajectoryActionResultConstPtr pt;
  ros::Duration timeout = traj.joint_trajectory.points.back().time_from_start;
  if(has_ctrl != 0)
  {
    // only do this if a controller exists - use a scaled timeout
    timeout = timeout*1.3;
    pt = ros::topic::waitForMessage<control_msgs::FollowJointTrajectoryActionResult>(controller_name + "result",node,timeout);
    if(pt)
      ROS_INFO_STREAM("ikControl::waitForExecution : received message - error_code=" << pt->result.error_code);
    else
      ROS_INFO_STREAM("ikControl::waitForExecution : timeout reached");
  }
  else
  {
    ROS_INFO_STREAM("ikControl::waitForExecution : waiting for at least " << timeout << " (trajectory total time)");
    timeout.sleep();
  }

  std::vector<std::string> joints = traj.joint_trajectory.joint_names;
  std::vector<double> q,Dq,goal_q;
  q.reserve(joints.size());
  Dq.reserve(joints.size());
  goal_q.reserve(joints.size());
  
  // (remember that the goal position is the last pose of the trajectory)
  for(auto q_i:traj.joint_trajectory.points.back().positions)
    goal_q.push_back(q_i);
  
  // goal size and joints size MUST be equal: check
  assert(goal_q.size() == joints.size());

  double vel,dist;
  bool good_stop = false;
  
  sensor_msgs::JointStateConstPtr joint_states;
  
  // wait until the robot is moving
  vel = 1.0 + velocity_threshold;
  while(vel > velocity_threshold)
  {
    q.clear();
    Dq.clear();
    vel = 0.0;
    dist = 0.0;
    
    // TODO: use one subscriber instead of waiting on single messages?
    joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",node,ros::Duration(3));
    bool joint_found;
    
    //get joint states
    for(auto joint:joints)
    {
      joint_found = false;
      for(int i=0; i<joint_states->name.size(); i++)
      {
	if(joint == joint_states->name.at(i))
	{
	  q.push_back(joint_states->position.at(i));
	  Dq.push_back(joint_states->velocity.at(i));
	  joint_found = true;
	  break;
	}
      }
      if(!joint_found)
      {
	ROS_ERROR("ikControl::waitForExecution : couldn't find requested joints!");
	return false;
      }
    }
    
    for(auto v:Dq)
      vel += std::norm(v);
    
    //if velocity < eps1
    if (vel<velocity_threshold)
    {
      for(int i=0; i<q.size(); i++)
	dist += std::norm(q.at(i) - goal_q.at(i));
      
      //if norm(fk(position) - goal) < eps2
      if (dist<position_threshold)
      {
	good_stop = true;
	break;
      }
      else
      {
        ROS_WARN_STREAM("ikControl::waitForExecution : vel=" << vel << " (< " << velocity_threshold << ") but dist=" << dist << " (>= " << position_threshold << ")");
	break;
      }
    }
    usleep(100000);
  }
  
  if(good_stop)
    ROS_INFO("ikControl::waitForExecution : exiting with good_stop OK");
  else
    ROS_WARN("ikControl::waitForExecution : exiting with error");
  return good_stop;
}

void ikControl::ik_check_thread(dual_manipulation_shared::ik_service::Request req)
{
  ROS_INFO("IKControl::ik_check_thread: Thread spawned! Computing IK for %s",req.ee_name.c_str());

  std_msgs::String msg;
  
  if(ik_check_capability_.manage_ik(req))
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }
  hand_pub.at(IK_CHECK_CAPABILITY).at(req.ee_name).publish(msg); //publish on a topic when the IK check is done

  busy.at(IK_CHECK_CAPABILITY).at(req.ee_name)=false;
  
  return;
}

void ikControl::planning_thread(dual_manipulation_shared::ik_service::Request req)
{
  
    ROS_INFO("IKControl::planning_thread: Thread spawned! Computing plan for %s",req.ee_name.c_str());
    
    // this connecs to a running instance of the move_group node
    move_group_interface::MoveGroup* localMoveGroup = moveGroups_.at(req.ee_name);
    
    std::cout << "IKControl::planning_thread: Planning for group " << group_map_.at(req.ee_name) << std::endl;
//     geometry_msgs::Pose current_pose = localMoveGroup->getCurrentPose().pose;
//     
//     std::cout << "pos [x y z]: " << current_pose.position.x << " " << current_pose.position.y << " " << current_pose.position.z << std::endl;
//     std::cout << "orient [x y z w]: "  << current_pose.orientation.x << " " << current_pose.orientation.y << " " << current_pose.orientation.z << " " << current_pose.orientation.w << std::endl;

    moveit::planning_interface::MoveItErrorCode error_code;

    std_msgs::String msg;
    
    bool target_set = false;
    if (req.ee_name != "both_hands")
    {
      target_set = localMoveGroup->setPoseTarget(req.ee_pose.at(0));
    }
    else
    {
      // // a workaround to move both arms and avoid collisions, as the simple following line doesn't work... it only moves the left hand
      // target_set = localMoveGroup->setPoseTarget(req.ee_pose.at(1),"right_hand_palm_link") && localMoveGroup->setPoseTarget(req.ee_pose.at(0),"left_hand_palm_link");
      
      moveit::planning_interface::MoveGroup::Plan tmpPlan;
      std::vector<double> left_joints, right_joints;
      
      target_set = moveGroups_.at("left_hand")->setPoseTarget(req.ee_pose.at(0));
      
      if(target_set)
      {
	error_code = moveGroups_.at("left_hand")->plan(tmpPlan);
	if(error_code.val != 1)
	{
	  ROS_WARN_STREAM("IKControl::planning_thread: sub-plan for left_hand FAILED!");
	  msg.data = "error";
	  hand_pub.at(PLAN_CAPABILITY).at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done
	  busy.at(PLAN_CAPABILITY).at(req.ee_name)=false;
	  return;
	}
	
	left_joints = tmpPlan.trajectory_.joint_trajectory.points.back().positions;
	
	std::cout << "left_joints = ";
	for(auto item:left_joints)
	  std::cout << item << "|";
	std::cout << std::endl;
      }
      
      target_set = target_set && moveGroups_.at("right_hand")->setPoseTarget(req.ee_pose.at(1));
      if (target_set)
      {
	moveit::core::RobotStatePtr current_state = moveGroups_.at("right_hand")->getCurrentState();
	current_state->setJointGroupPositions("left_hand_arm",left_joints);
	
	// const moveit::core::JointModelGroup *jmg = current_state->getJointModelGroup("right_hand_arm");
	// for(auto item:jmg->getJointModelNames())
	//   std::cout << item << " | ";
	// std::cout << std::endl;
	
	error_code = moveGroups_.at("right_hand")->plan(tmpPlan);
	if(error_code.val != 1)
	{
	  ROS_WARN_STREAM("IKControl::planning_thread: sub-plan for right_hand FAILED!");
	  msg.data = "error";
	  hand_pub.at(PLAN_CAPABILITY).at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done
	  busy.at(PLAN_CAPABILITY).at(req.ee_name)=false;
	  return;
	}
	
	right_joints = tmpPlan.trajectory_.joint_trajectory.points.back().positions;
	
	std::cout << "right_joints = ";
	for(auto item:right_joints)
	  std::cout << item << "|";
	std::cout << std::endl;
      }

      std::vector<double> bimanual_joints;
      bimanual_joints.resize(localMoveGroup->getCurrentJointValues().size() - left_joints.size() - right_joints.size());

      bimanual_joints.insert(bimanual_joints.end(),left_joints.begin(),left_joints.end());
      bimanual_joints.insert(bimanual_joints.end(),right_joints.begin(),right_joints.end());

      target_set = target_set && localMoveGroup->setJointValueTarget( bimanual_joints );
    }
    
    if ( target_set )
    {
      ROS_INFO_STREAM("IKControl::planning_thread: Target set correctly!" << std::endl);
    }
    else
    {
      ROS_WARN_STREAM("IKControl::planning_thread: Unable to set target pose\n");
      msg.data = "error";
      hand_pub.at(PLAN_CAPABILITY).at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done

      busy.at(PLAN_CAPABILITY).at(req.ee_name)=false;

      return;
    }
    
    moveit::planning_interface::MoveGroup::Plan* movePlan = &(movePlans_.at(req.ee_name));
    
    localMoveGroup->setStartStateToCurrentState();
    error_code = localMoveGroup->plan(*movePlan);
    
    ROS_INFO_STREAM("movePlan traj size: " << movePlan->trajectory_.joint_trajectory.points.size() << std::endl);
    for (int i=0; i<movePlan->trajectory_.joint_trajectory.points.size(); ++i)
    {
      ROS_DEBUG_STREAM(movePlan->trajectory_.joint_trajectory.points.at(i) << std::endl);
      // std::cout << movePlan->trajectory_.joint_trajectory.points.at(i) << std::endl;
    }
    
    ROS_DEBUG_STREAM("pos [x y z]: " << req.ee_pose.at(0).position.x << " " << req.ee_pose.at(0).position.y << " " << req.ee_pose.at(0).position.z << std::endl);
    ROS_DEBUG_STREAM("orient [x y z w]: "  << req.ee_pose.at(0).orientation.x << " " << req.ee_pose.at(0).orientation.y << " " << req.ee_pose.at(0).orientation.z << " " << req.ee_pose.at(0).orientation.w << std::endl);

    if (error_code.val == 1)
    {
      msg.data = "done";
    }
    else
    {
      msg.data = "error";
    }
    
    hand_pub.at(PLAN_CAPABILITY).at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done
  
    busy.at(PLAN_CAPABILITY).at(req.ee_name)=false;
    
    return;
}

void ikControl::execute_plan(dual_manipulation_shared::ik_service::Request req)
{
  ROS_INFO("IKControl::execute_plan: Executing plan for %s",req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;
  
  // old execution method: does not allow for two trajectories at the same time
  error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlans_.at(req.ee_name));
  
  bool good_stop = waitForExecution(req.ee_name,movePlans_.at(req.ee_name).trajectory_);

  std_msgs::String msg;
  
  if(good_stop)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }
  hand_pub.at(MOVE_CAPABILITY).at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done

  busy.at(MOVE_CAPABILITY).at(req.ee_name)=false;
  
  return;
}

bool ikControl::is_free_make_busy(std::string ee_name, std::string capability)
{
    std::unique_lock<std::mutex>(map_mutex_);

    if(!busy.count(capability))
    {
	ROS_ERROR("IKControl::perform_ik: Unknown capability %s, returning",capability.c_str());
	return false;
    }

    if(!busy.at(capability).count(ee_name))
    {
	ROS_ERROR("IKControl::perform_ik: Unknown end effector %s, returning",ee_name.c_str());
	return false;
    }

    bool is_busy = false;

    // if I'm checking for a tree
    if(std::find(tree_names_list_.begin(),tree_names_list_.end(),ee_name) != tree_names_list_.end())
    {
      // if it's a capability which is not implemented yet for trees
      // TODO: take out IK_CHECK_CAPABILITY once it's available
      if((capability == IK_CHECK_CAPABILITY) || (capability == GRASP_CAPABILITY) || (capability == UNGRASP_CAPABILITY))
      {
	  ROS_ERROR("IKControl::perform_ik: Perform %s commands for each end-effector separately (tree version not implemented yet)! Returning",capability.c_str());
	  return false;
      }

      // check if any part of the tree is busy
      // TODO: this is probably not necessary for all capabilities, but keep it coherent for now
      std::vector<std::string>& chains = tree_composition_.at(ee_name);
      for(auto chain:chains)
	is_busy = is_busy || busy.at(capability).at(chain);
    }
    // if I'm checking for a chain, just be sure that its tree (if exists) is free
    // TODO: this is probably not necessary for all capabilities, but keep it coherent for now
    else
    {
      for(auto tree:tree_names_list_)
	if(std::find(tree_composition_.at(tree).begin(),tree_composition_.at(tree).end(),ee_name) != tree_composition_.at(tree).end())
	{
	  is_busy = busy.at(capability).at(tree);
	  break;
	}
    }
    
    // check whether the end-effector is free, and in case make it busy
    is_busy = is_busy || busy.at(capability).at(ee_name);
    if(!is_busy)
      busy.at(capability).at(ee_name) = true;
    
    return (!is_busy);
}

bool ikControl::perform_ik(dual_manipulation_shared::ik_service::Request& req)
{
    std::cout << "perform_ik : req.command = " << req.command << std::endl;
    if(req.command == "stop")
    {
      this->stop();
      return true;
    }
    if(req.command == "free_all")
    {
      this->free_all();
      return true;
    }
    
    if(is_free_make_busy(req.ee_name,req.command))
    {
	std::thread* th;
	if(req.command == PLAN_CAPABILITY)
	{
	  th = new std::thread(&ikControl::planning_thread,this, req);
	}
	else if(req.command == IK_CHECK_CAPABILITY)
	{
	  th = new std::thread(&ikControl::ik_check_thread,this, req);
	}
	else if(req.command == MOVE_CAPABILITY)
	{
	  th = new std::thread(&ikControl::execute_plan,this, req);
	}
	else if(req.command == HOME_CAPABILITY)
	{
	  th = new std::thread(&ikControl::simple_homing,this, req.ee_name);
	}
	else if(req.command == GRASP_CAPABILITY)
	{
	  th = new std::thread(&ikControl::grasp,this, req);
	}
	else if(req.command == UNGRASP_CAPABILITY)
	{
	  th = new std::thread(&ikControl::ungrasp,this, req);
	}
	else
	{
	  ROS_WARN("IKControl::perform_ik: Unknown command: %s",req.command.c_str());
	  return false;
	}
	used_threads_.push_back(th);
    }
    else
    {
	ROS_WARN("IKControl::perform_ik: Already performing a %s ik_service",req.ee_name.c_str());
	return false;
    }

    return true;
}

ikControl::~ikControl()
{
    for(auto group:moveGroups_)
      delete group.second;
    
    for(int i=0; i<used_threads_.size(); i++)
      delete used_threads_.at(i);
}

bool ikControl::moveHand(std::string& hand, std::vector< double >& q, std::vector< double >& t)
{
  trajectory_msgs::JointTrajectory grasp_traj;
  
  // // do not fill the header if you're using different computers

  grasp_traj.joint_names.push_back(hand_actuated_joint_.at(hand));
  
  if (t.size() != q.size())
  {
    ROS_WARN("IKControl::moveHand: timing vector size non compatible with joint vector size, using a default timing of 1 second");
    t.clear();
    for (int i=0; i<q.size(); ++i)
      t.push_back(1.0/q.size()*i);
  }
  
  trajectory_msgs::JointTrajectoryPoint tmp_traj;
  tmp_traj.positions.reserve(1);
  
  for (int i=0; i<q.size(); ++i)
  {
    tmp_traj.positions.clear();
    tmp_traj.positions.push_back(q.at(i));
    tmp_traj.time_from_start = ros::Duration(t.at(i));
  
    grasp_traj.points.push_back(tmp_traj);
  }
  
  hand_synergy_pub_.at(hand).publish(grasp_traj);

  return true;
}

bool ikControl::moveHand(std::string& hand, trajectory_msgs::JointTrajectory& grasp_traj)
{
  hand_synergy_pub_.at(hand).publish(grasp_traj);
  
  return true;
}

void ikControl::simple_homing(std::string ee_name)
{
  ROS_INFO("IKControl::simple_homing: going back home...");

  // if the group is moving, stop it
  moveGroups_.at(ee_name)->stop();
  moveGroups_.at(ee_name)->setNamedTarget( group_map_.at(ee_name) + "_home" );
  moveGroups_.at(ee_name)->setStartStateToCurrentState();

  std_msgs::String msg;
  
  moveit::planning_interface::MoveItErrorCode error_code = moveGroups_.at(ee_name)->plan(movePlans_.at(ee_name));
  if(error_code.val != 1)
  {
    ROS_ERROR_STREAM("ikControl::simple_homing : unable to plan for \"" << group_map_.at(ee_name) << "_home\", returning");
    msg.data = "error";
    hand_pub.at(HOME_CAPABILITY).at(ee_name).publish(msg);
    busy.at(HOME_CAPABILITY).at(ee_name) = false;
    return;
  }
  
  error_code = moveGroups_.at(ee_name)->asyncExecute(movePlans_.at(ee_name));
  if(error_code.val != 1)
  {
    ROS_ERROR_STREAM("ikControl::simple_homing : unable to forward \"" << group_map_.at(ee_name) << "_home\" trajectory to the controller, returning");
    msg.data = "error";
    hand_pub.at(HOME_CAPABILITY).at(ee_name).publish(msg);
    busy.at(HOME_CAPABILITY).at(ee_name) = false;
    return;
  }
  
  // also open the hand(s) we're moving home, but don't wait for it(them)
  std::vector <double > q = {0.0};
  std::vector <double > t = {1.0/hand_max_velocity};
  if (ee_name != "both_hands")
    moveHand(ee_name,q,t);
  else
  {
    std::string hand_name = "right_hand";
    moveHand(hand_name,q,t);
    hand_name = "left_hand";
    moveHand(hand_name,q,t);
  }
  
  bool good_stop = waitForExecution(ee_name,movePlans_.at(ee_name).trajectory_);
  if(!good_stop)
  {
    msg.data = "error";
  }
  else
  {
    msg.data = "done";
  }
  hand_pub.at(HOME_CAPABILITY).at(ee_name).publish(msg);
  busy.at(HOME_CAPABILITY).at(ee_name) = false;
  
  return;
}

void ikControl::grasp(dual_manipulation_shared::ik_service::Request req)
{
// // //   TODO: check if this is actually necessary as we're not using planning, but it may check for collisions on its own
// // //   removeObject(req.attObject.object.id);
  
  ROS_INFO("IKControl::grasp: %s with %s",req.attObject.object.id.c_str(),req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;

  std_msgs::String msg;
  
  // // get timed trajectory from waypoints
  moveit_msgs::RobotTrajectory trajectory;
  if(!computeTrajectoryFromWPs(trajectory,req.ee_pose,moveGroups_.at(req.ee_name)))
  {
    ROS_ERROR("ikControl::grasp : unable to get trajectory from waypoints, returning");
    msg.data = "error";
    hand_pub.at(GRASP_CAPABILITY).at(req.ee_name).publish(msg);
    return;
  }

  // // align trajectories in time and check hand velocity limits
  computeHandTiming(trajectory,req);
  
  // // do not fill the header if you're using different computers
  
  // // execution of approach
  movePlans_.at(req.ee_name).trajectory_ = trajectory;
  error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlans_.at(req.ee_name));
  if (error_code.val != 1)
  {
    ROS_ERROR("ikControl::grasp : unable to send trajectory to the controller, returning");
    msg.data = "error";
    hand_pub.at(GRASP_CAPABILITY).at(req.ee_name).publish(msg);
    return;
  }
  
#ifndef SIMPLE_GRASP
  moveHand(req.ee_name,req.grasp_trajectory);
#endif
  
  // // wait for approach
  bool good_stop = waitForExecution(req.ee_name,movePlans_.at(req.ee_name).trajectory_);
  // I didn't make it
  if (!good_stop)
  {
    ROS_ERROR("ikControl::grasp : unable to execute approach trajectory, returning");
    msg.data = "error";
    hand_pub.at(GRASP_CAPABILITY).at(req.ee_name).publish(msg);
    return;
  }

#ifdef SIMPLE_GRASP
  // // moveHand
  std::vector <double > q = {0.4,1.0};
  std::vector <double > t = {0.4/hand_max_velocity,0.5+1.0/hand_max_velocity};
  moveHand(req.ee_name,q,t);
  req.grasp_trajectory.points.back().positions.at(0) = 1.0;
#endif
  
  // // wait for hand moved
  good_stop = waitForHandMoved(req.ee_name,req.grasp_trajectory.points.back().positions.at(0));
  // I didn't make it
  if (!good_stop)
  {
    ROS_ERROR("ikControl::grasp : unable to execute grasp trajectory, returning");
    msg.data = "error";
    hand_pub.at(GRASP_CAPABILITY).at(req.ee_name).publish(msg);
    return;
  }

  // // attach object (only if everything went smoothly)
  //check whether the object was present, and in case remove it from the environment
  dual_manipulation_shared::scene_object_service::Request req_obj;
  req_obj.command = "attach";
  req_obj.object_id = req.attObject.object.id;
  req_obj.attObject = req.attObject;
  // insert the links which does not constitute a collision
  req_obj.attObject.touch_links.insert(req_obj.attObject.touch_links.begin(),allowed_collisions_.at(req.ee_name).begin(),allowed_collisions_.at(req.ee_name).end());
  req_obj.object_db_id = req.object_db_id;
  scene_object_manager_.manage_object(req_obj);
  
  // we made it!
  msg.data = "done";
  hand_pub.at(GRASP_CAPABILITY).at(req.ee_name).publish(msg);
  busy.at(GRASP_CAPABILITY).at(req.ee_name) = false;
  
  return;
}

void ikControl::ungrasp(dual_manipulation_shared::ik_service::Request req)
{
  ROS_INFO("IKControl::ungrasp: %s from %s",req.attObject.object.id.c_str(),req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;

  std_msgs::String msg;
  
  // // get timed trajectory from waypoints
  moveit_msgs::RobotTrajectory trajectory;
  if(!computeTrajectoryFromWPs(trajectory,req.ee_pose,moveGroups_.at(req.ee_name)))
  {
    ROS_ERROR("ikControl::grasp : unable to get trajectory from waypoints, returning");
    msg.data = "error";
    hand_pub.at(UNGRASP_CAPABILITY).at(req.ee_name).publish(msg);
    return;
  }

  // // align trajectories in time and check hand velocity limits
  computeHandTiming(trajectory,req);
  
  // // do not fill the header if you're using different computers
  
  bool good_stop = false;
  
#ifndef SIMPLE_GRASP
  // // moveHand
  moveHand(req.ee_name,req.grasp_trajectory);
#elif SIMPLE_GRASP
  // // moveHand
  std::vector <double > q = {0.0};
  std::vector <double > t = {1.0/hand_max_velocity};
  moveHand(req.ee_name,q,t);
  // // wait for hand moved
  good_stop = waitForHandMoved(req.ee_name,0.0);
  // I didn't make it
  if (!good_stop)
  {
    ROS_ERROR("ikControl::ungrasp : unable to execute ungrasp trajectory, returning");
    msg.data = "error";
    hand_pub.at(UNGRASP_CAPABILITY).at(req.ee_name).publish(msg);
    return;
  }
#endif

  // // execution of retreat
  movePlans_.at(req.ee_name).trajectory_ = trajectory;
  error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlans_.at(req.ee_name));
  if (error_code.val != 1)
  {
    ROS_ERROR("ikControl::ungrasp : unable to send trajectory to the controller, returning");
    msg.data = "error";
    hand_pub.at(UNGRASP_CAPABILITY).at(req.ee_name).publish(msg);
    return;
  }
  
  // // wait for retreat
  good_stop = waitForExecution(req.ee_name,movePlans_.at(req.ee_name).trajectory_);
  // I didn't make it
  if (!good_stop)
  {
    ROS_ERROR("ikControl::ungrasp : unable to execute retreat trajectory, returning");
    msg.data = "error";
    hand_pub.at(UNGRASP_CAPABILITY).at(req.ee_name).publish(msg);
    return;
  }
  
#ifndef SIMPLE_GRASP
  // // wait for hand moved
  good_stop = waitForHandMoved(req.ee_name,req.grasp_trajectory.points.back().positions.at(0));
  // I didn't make it
  if (!good_stop)
  {
    ROS_ERROR("ikControl::ungrasp : unable to execute ungrasp trajectory, returning");
    msg.data = "error";
    hand_pub.at(UNGRASP_CAPABILITY).at(req.ee_name).publish(msg);
    return;
  }
#endif

  // put the object back in the scene
  dual_manipulation_shared::scene_object_service::Request req_scene;
  req_scene.command = "detach";
  req_scene.attObject = req.attObject;
  req_scene.object_id = req.attObject.object.id;
  req_scene.object_db_id = req.object_db_id;

  if(!scene_object_manager_.manage_object(req_scene))
  {
    ROS_WARN("IKControl::ungrasp: object with ID \"%s\" is not grasped by %s. Performing ungrasp action anyway",req.attObject.object.id.c_str(),req.ee_name.c_str());
  }

  msg.data = "done";
  hand_pub.at(UNGRASP_CAPABILITY).at(req.ee_name).publish(msg);
  busy.at(UNGRASP_CAPABILITY).at(req.ee_name) = false;
  
  return;
}

