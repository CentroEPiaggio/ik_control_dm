#include "ik_control.h"
#include "trajectory_utils.h"
#include <dual_manipulation_shared/parsing_utils.h>
#include <dual_manipulation_shared/ik_response.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/String.h>

#define SIMPLE_GRASP 1
#define CLASS_NAMESPACE "ikControl::"
#define DEFAULT_MAX_PLANNING_ATTEMPTS 1

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
    clik_threshold_ = 0.1;
    epsilon_ = 0.001;
    
    kinematics_only_ = false;
    
    allowed_collision_prefixes_.clear();
    allowed_collision_prefixes_["left_hand"] = std::vector<std::string>({"left_hand","left_arm_7_link"});
    allowed_collision_prefixes_["right_hand"] = std::vector<std::string>({"right_hand","right_arm_7_link"});
    
    // planner parameters
    planner_id_ = "RRTstarkConfigDefault";
    planning_time_ = 2.0;
    backup_planner_id_ = "RRTConnectkConfigDefault";
    backup_planning_time_ = 5.0;
    max_planning_attempts_ = DEFAULT_MAX_PLANNING_ATTEMPTS;
    goal_position_tolerance_ = 0.005;
    goal_orientation_tolerance_ = 0.005;
    goal_joint_tolerance_ = 0.005;
    ws_bounds_.assign({-1.2,-1.5,0.1,0.2,1.5,1.5});
    
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
    
    movement_end_time_ = ros::Time::now();

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
      delete ik_check_;
      delete position_only_ik_check_;
      delete ik_check_legacy_;
      
      setParameterDependentVariables();
    }
}

void ikControl::setParameterDependentVariables()
{
  for(auto group_name:group_map_)
  {
    moveGroups_[group_name.first] = new move_group_interface::MoveGroup( group_name.second, boost::shared_ptr<tf::Transformer>(), ros::Duration(5, 0) );
    movePlans_[group_name.first];

    for(auto capability:capabilities_.name)
    {
      hand_pub[capability.first][group_name.first] = node.advertise<dual_manipulation_shared::ik_response>("/ik_control/" + group_name.first + "/" + capabilities_.msg[capability.first],1,this);
      busy[capabilities_.type[capability.first]][group_name.first] = false;
      ROS_INFO_STREAM("hand_pub[" << capability.second << "][" << group_name.first << "] => /ik_control/" + group_name.first + "/" + capabilities_.msg[capability.first]);
    }
  }
  
  for(auto item:moveGroups_)
  {
    item.second->setPlannerId(planner_id_);
    item.second->setPlanningTime(planning_time_);
    item.second->setNumPlanningAttempts(max_planning_attempts_);
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
  
  // build robotModels and robotStates
  // NOTE: this way, they never actually change - consider moving them in the constructor
  ros::NodeHandle n("~"); // a private NodeHandle is needed to set parameters for KDLKinematicsPlugin
  n.setParam("epsilon",epsilon_);
  robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
  robot_model_ = robot_model_loader_->getModel();
  // set parameters of the private nodeHandle to load a robotModel with position only IK
  for(auto jmg:group_map_)
    n.setParam(jmg.second + "/position_only_ik",true);
  position_only_ik_robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
  position_only_ik_robot_model_ = position_only_ik_robot_model_loader_->getModel();
  for(auto jmg:group_map_)
    n.setParam(jmg.second + "/position_only_ik",false);
  ik_check_ = new ikCheckCapability(robot_model_);
  position_only_ik_check_ = new ikCheckCapability(position_only_ik_robot_model_);
  ik_check_legacy_ = new ikCheckCapability(robot_model_);
  target_rs_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_));
  planning_init_rs_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_));
  
  reset_robot_state(target_rs_);
  reset_robot_state(planning_init_rs_);
}

void ikControl::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    parseSingleParameter(params,position_threshold,"position_threshold");
    parseSingleParameter(params,velocity_threshold,"velocity_threshold");
    parseSingleParameter(params,hand_max_velocity,"hand_max_velocity");
    parseSingleParameter(params,hand_position_threshold,"hand_position_threshold");
    parseSingleParameter(params,kinematics_only_,"kinematics_only");
    parseSingleParameter(params,clik_threshold_,"clik_threshold");
    parseSingleParameter(params,epsilon_,"epsilon");

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
    std::vector<std::string> tree_names_tmp;
    tree_names_tmp.swap(tree_names_list_);
    for(auto tree:tree_names_tmp)
      if(!tree_composition_.count(tree) || tree_composition_.at(tree).size() == 0)
	ROS_WARN_STREAM("No composition is specified for tree '" << tree << "': check the yaml configuration.");
      else
	tree_names_list_.push_back(tree);
    
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
      parseSingleParameter(params["motion_planner"],backup_planner_id_,"backup_planner_id");
      parseSingleParameter(params["motion_planner"],backup_planning_time_,"backup_planning_time");
      parseSingleParameter(params["motion_planner"],max_planning_attempts_,"max_planning_attempts");
      if(max_planning_attempts_ <= 0) max_planning_attempts_ = DEFAULT_MAX_PLANNING_ATTEMPTS;
      parseSingleParameter(params["motion_planner"],goal_position_tolerance_,"goal_position_tolerance");
      parseSingleParameter(params["motion_planner"],goal_orientation_tolerance_,"goal_orientation_tolerance");
      parseSingleParameter(params["motion_planner"],goal_joint_tolerance_,"goal_joint_tolerance");
      parseSingleParameter(params["motion_planner"],ws_bounds_,"workspace_bounds",6);
    }
}

bool ikControl::manage_object(dual_manipulation_shared::scene_object_service::Request& req)
{
    std::unique_lock<std::mutex>(scene_object_mutex_);
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
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now() + ros::Duration(1.0);
    end_time_mutex_.unlock();
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
  end_time_mutex_.lock();
  movement_end_time_ = ros::Time::now() + timeout;
  end_time_mutex_.unlock();
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
  {
    ROS_WARN("ikControl::waitForExecution : exiting with error");
    reset();
  }
  return good_stop;
}

void ikControl::ik_check_thread(dual_manipulation_shared::ik_service::Request req)
{
  ik_control_capabilities local_capability = ik_control_capabilities::IK_CHECK;
  
  ROS_INFO("IKControl::ik_check_thread: Thread spawned! Computing IK for %s",req.ee_name.c_str());

  dual_manipulation_shared::ik_response msg;
  msg.seq=req.seq;
  
  // NOTE: this lock is to perform both operations at the same time, but it's not necessary for thread-safety
  ikCheck_mutex_.lock();
  ik_check_legacy_->reset_robot_state();
  std::vector<std::vector<double>> sol;
  bool ik_ok = ik_check_legacy_->find_group_ik(req.ee_name,req.ee_pose,sol);
  ikCheck_mutex_.unlock();
  
  if(ik_ok)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }
  hand_pub.at(local_capability).at(req.ee_name).publish(msg); //publish on a topic when the IK check is done

  map_mutex_.lock();
  busy.at(capabilities_.type.at(local_capability)).at(req.ee_name)=false;
  map_mutex_.unlock();
  
  return;
}

void ikControl::planning_thread(dual_manipulation_shared::ik_service::Request req, bool check_collisions, bool use_clik)
{
    ik_control_capabilities local_capability;
    if(check_collisions && !use_clik)
      local_capability = ik_control_capabilities::PLAN;
    else if(!check_collisions && !use_clik)
      local_capability = ik_control_capabilities::PLAN_NO_COLLISION;
    else if(check_collisions && use_clik)
      local_capability = ik_control_capabilities::PLAN_BEST_EFFORT;
    else if(!check_collisions && use_clik)
      local_capability = ik_control_capabilities::PLAN_BEST_EFFORT_NO_COLLISION;
  
    ROS_INFO("IKControl::planning_thread: Thread spawned! Computing plan for %s",req.ee_name.c_str());
    
    move_group_interface::MoveGroup* localMoveGroup;
    std::string group_name;
    std::map<std::string,ik_target> local_targets;
    map_mutex_.lock();
    group_name = group_map_.at(req.ee_name);
    
    // in case I'm looking for a single target
    if(targets_.count(req.ee_name) != 0)
    {
      local_targets[req.ee_name] = targets_.at(req.ee_name);
      targets_.erase(req.ee_name);
    }
    // here, I'm looking for a possible composition
    else if(std::find(tree_names_list_.begin(),tree_names_list_.end(),req.ee_name) != tree_names_list_.end())
    {
      for(auto chain:tree_composition_.at(req.ee_name))
	if(targets_.count(chain))
	{
	  local_targets[chain] = targets_.at(chain);
	  targets_.erase(chain);
	}
    }
    // NO target to be set...!
    else
    {
      ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : no target needs to be set...");
    }
    map_mutex_.unlock();
    
    std::cout << "IKControl::planning_thread: Planning for group " << group_name << std::endl;
//     geometry_msgs::Pose current_pose = localMoveGroup->getCurrentPose().pose;
//     
//     std::cout << "pos [x y z]: " << current_pose.position.x << " " << current_pose.position.y << " " << current_pose.position.z << std::endl;
//     std::cout << "orient [x y z w]: "  << current_pose.orientation.x << " " << current_pose.orientation.y << " " << current_pose.orientation.z << " " << current_pose.orientation.w << std::endl;

    moveit::planning_interface::MoveItErrorCode error_code;

    dual_manipulation_shared::ik_response msg;
    msg.seq=req.seq;
    bool target_set = true;
    // if I'm using CLIK it means that I want to get as close as possible to my target, so, if needed, don't care about the orientation
    bool position_only_ik = use_clik;
    
    // first set all NAMED_TARGET's, then all POSE_TARGET's; for JOINT_TARGET's issue an error
    for(auto target_p:local_targets)
    {
      ik_target& target(target_p.second);
      if(target.type == ik_target_type::NAMED_TARGET)
	target_set = target_set && set_target(target.ee_name,target.target_name);
    }
    for(auto target_p:local_targets)
    {
      ik_target& target(target_p.second);
      if(target.type == ik_target_type::JOINT_TARGET)
	ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : joint-value targets are not supported yet! Ignoring the target set for " << target.ee_name);
	// target_set = target_set && set_target(target.ee_name,target.joints);
    }
    for(auto target_p:local_targets)
    {
      ik_target& target(target_p.second);
      if(target.type == ik_target_type::POSE_TARGET)
	target_set = target_set && set_target(target.ee_name,target.ee_poses,check_collisions,use_clik,position_only_ik);
    }
    
    // get and set the complete joint value target
    if(target_set)
    {
      robotState_mutex_.lock();
      moveit::core::RobotState rs_target(ik_check_->get_robot_state());
      moveit::core::RobotState rs_init(*planning_init_rs_);
      robotState_mutex_.unlock();
      
      moveGroups_mutex_.lock();
      localMoveGroup = moveGroups_.at(req.ee_name);
      target_set = localMoveGroup->setJointValueTarget(rs_target);
      localMoveGroup->setStartState(rs_init);
      moveGroups_mutex_.unlock();
    }
    
    if ( target_set )
    {
      ROS_INFO_STREAM("IKControl::planning_thread: Target set correctly!" << std::endl);
    }
    else
    {
      ROS_WARN_STREAM("IKControl::planning_thread: Unable to set target pose\n");
      msg.data = "error";
      hand_pub.at(local_capability).at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done

      map_mutex_.lock();
      busy.at(capabilities_.type.at(local_capability)).at(req.ee_name)=false;
      map_mutex_.unlock();

      return;
    }
    
    moveit::planning_interface::MoveGroup::Plan movePlan;
    
    if(check_collisions)
    {
      double plan_time;
      ros::Time tmp;
      end_time_mutex_.lock();
      tmp = movement_end_time_;
      end_time_mutex_.unlock();
      // wait for the execution to be initialized - sleep 5ms if an execution function has been called but has not passed to actual execution yet
      while(tmp == ros::Time(0))
      {
	usleep(5000);
	end_time_mutex_.lock();
	tmp = movement_end_time_;
	end_time_mutex_.unlock();
      }
      ros::Duration residual_move_time = tmp - ros::Time::now();
      plan_time = std::max(planning_time_,residual_move_time.toSec());
      ROS_INFO_STREAM(CLASS_NAMESPACE << __func__ << " : will use " << plan_time << "s planning time, max between default " << planning_time_ << "s and residual movemenent time " << residual_move_time.toSec() << "s");
      localMoveGroup->setPlanningTime(plan_time);
      error_code = localMoveGroup->plan(movePlan);
      
      if(error_code.val != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
	ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : unable to plan with \'" << planner_id_ << "\' with timeout of " << plan_time << "s, trying once more with \'" << backup_planner_id_ << "\' and timeout of " << backup_planning_time_ << "s");
	// attempt a chance planning with a random algorithm and a longer waiting: if it fails, it should fail
	localMoveGroup->setPlannerId(backup_planner_id_);
	localMoveGroup->setPlanningTime(backup_planning_time_);
	error_code = localMoveGroup->plan(movePlan);
	// reset the planner ID
	localMoveGroup->setPlannerId(planner_id_);
      }
    }
    else
    {
      // get timed trajectory from waypoints
      moveit_msgs::RobotTrajectory& trajectory(movePlan.trajectory_);
      robotState_mutex_.lock();
      bool add_wp_ok = add_wp_to_traj(planning_init_rs_,group_name,trajectory) && add_wp_to_traj(target_rs_,group_name,trajectory);
      robotState_mutex_.unlock();
      if(add_wp_ok)
      {
	error_code.val = moveit::planning_interface::MoveItErrorCode::SUCCESS;
      }
      else
	error_code.val = moveit::planning_interface::MoveItErrorCode::NO_IK_SOLUTION;
    }
    
    ROS_INFO_STREAM("movePlan traj size: " << movePlan.trajectory_.joint_trajectory.points.size() << std::endl);
    for (int i=0; i<movePlan.trajectory_.joint_trajectory.points.size(); ++i)
    {
      ROS_DEBUG_STREAM(movePlan.trajectory_.joint_trajectory.points.at(i) << std::endl);
    }
    
    ROS_DEBUG_STREAM("pos [x y z]: " << req.ee_pose.at(0).position.x << " " << req.ee_pose.at(0).position.y << " " << req.ee_pose.at(0).position.z << std::endl);
    ROS_DEBUG_STREAM("orient [x y z w]: "  << req.ee_pose.at(0).orientation.x << " " << req.ee_pose.at(0).orientation.y << " " << req.ee_pose.at(0).orientation.z << " " << req.ee_pose.at(0).orientation.w << std::endl);

    if (error_code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      // // TODO: why not push_back the trajectory and get a new time-parametrization? the movePlan could be reset from the execution, in this way both collision and no-collision plans could be combined
      // movePlans_mutex_.lock();
      // moveit_msgs::RobotTrajectory tmp_traj = movePlans_.at(req.ee_name).trajectory_;
      // movePlans_mutex_.unlock();
      // robotState_mutex_.lock();
      // append_trajectories(planning_init_rs_,tmp_traj,movePlan.trajectory_);
      // robotState_mutex_.unlock();
      // std::swap(movePlan.trajectory_,tmp_traj);
      
      msg.data = "done";
      movePlans_mutex_.lock();
      movePlans_.at(req.ee_name) = movePlan;
      movePlans_mutex_.unlock();
      
      reset_robot_state(planning_init_rs_,req.ee_name,movePlan.trajectory_);
    }
    else
    {
      msg.data = "error";
    }
    
    hand_pub.at(local_capability).at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done
  
    map_mutex_.lock();
    busy.at(capabilities_.type.at(local_capability)).at(req.ee_name)=false;
    map_mutex_.unlock();
    
    return;
}

void ikControl::execute_plan(dual_manipulation_shared::ik_service::Request req)
{
  ik_control_capabilities local_capability = ik_control_capabilities::MOVE;
  end_time_mutex_.lock();
  movement_end_time_ = ros::Time(0);
  end_time_mutex_.unlock();
  
  ROS_INFO("IKControl::execute_plan: Executing plan for %s",req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;
  moveit::planning_interface::MoveGroup::Plan movePlan;
  
  movePlans_mutex_.lock();
  //NOTE: to be sure that no other execution is tried using this movePlan, use swap
  std::swap(movePlan,movePlans_.at(req.ee_name));
  movePlans_mutex_.unlock();
  
  // old execution method: does not allow for two trajectories at the same time
  moveGroups_mutex_.lock();
  error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlan);
  moveGroups_mutex_.unlock();
  
  bool good_stop = waitForExecution(req.ee_name,movePlan.trajectory_);

  dual_manipulation_shared::ik_response msg;
  msg.seq=req.seq;
  
  if(good_stop)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }
  hand_pub.at(local_capability).at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done

  map_mutex_.lock();
  busy.at(capabilities_.type.at(local_capability)).at(req.ee_name)=false;
  map_mutex_.unlock();
  
  return;
}

bool ikControl::is_free_make_busy(std::string ee_name, std::string capability_name)
{
    std::unique_lock<std::mutex>(map_mutex_);

    if(!capabilities_.from_name.count(capability_name))
    {
	ROS_ERROR("IKControl::perform_ik: Unknown capability %s, returning",capability_name.c_str());
	return false;
    }

    ik_control_capability_types capability;
    capability = capabilities_.type.at(capabilities_.from_name.at(capability_name));
    
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
      if(!capabilities_.implemented_for_trees.at(capability))
      {
	  ROS_ERROR("IKControl::perform_ik: Perform %s commands for each end-effector separately (tree version not implemented yet)! Returning",capability_name.c_str());
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
    // NOTE: if more than a single tree has that chain, the check should continue for all trees
    else
    {
      for(auto tree:tree_names_list_)
	if(std::find(tree_composition_.at(tree).begin(),tree_composition_.at(tree).end(),ee_name) != tree_composition_.at(tree).end())
	{
	  is_busy = is_busy || busy.at(capability).at(tree);
	  continue;
	}
    }
    
    // check whether the end-effector is free, and in case make it busy
    is_busy = is_busy || busy.at(capability).at(ee_name);
    if(!is_busy)
      busy.at(capability).at(ee_name) = true;
    else
      ROS_WARN_STREAM("IKControl::perform_ik: Already performing an ik_service of type " << capability_name << " for group " << ee_name);
    
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
	if(capabilities_.type.at(capabilities_.from_name.at(req.command)) == ik_control_capability_types::PLAN)
	{
	  bool check_collisions, use_clik;
	  if(req.command == capabilities_.name[ik_control_capabilities::PLAN])
	  {
	    check_collisions = true;
	    use_clik = false;
	  }
	  else if(req.command == capabilities_.name[ik_control_capabilities::PLAN_NO_COLLISION])
	  {
	    check_collisions = false;
	    use_clik = false;
	  }
	  else if(req.command == capabilities_.name[ik_control_capabilities::PLAN_BEST_EFFORT])
	  {
	    check_collisions = true;
	    use_clik = true;
	  }
	  else if(req.command == capabilities_.name[ik_control_capabilities::PLAN_BEST_EFFORT_NO_COLLISION])
	  {
	    check_collisions = false;
	    use_clik = true;
	  }
	  else
	  {
	    ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : this planning capability is not implemented yet!!!");
	    return false;
	  }
	  th = new std::thread(&ikControl::planning_thread,this, req, check_collisions, use_clik);
	}
	else if(req.command == capabilities_.name[ik_control_capabilities::IK_CHECK])
	{
	  th = new std::thread(&ikControl::ik_check_thread,this, req);
	}
	else if(req.command == capabilities_.name[ik_control_capabilities::MOVE])
	{
	  th = new std::thread(&ikControl::execute_plan,this, req);
	}
	else if(req.command == capabilities_.name[ik_control_capabilities::HOME])
	{
	  th = new std::thread(&ikControl::simple_homing,this, req);
	}
	else if(req.command == capabilities_.name[ik_control_capabilities::GRASP])
	{
	  th = new std::thread(&ikControl::grasp,this, req);
	}
	else if(req.command == capabilities_.name[ik_control_capabilities::UNGRASP])
	{
	  th = new std::thread(&ikControl::ungrasp,this, req);
	}
	else if(req.command == capabilities_.name[ik_control_capabilities::SET_TARGET])
	{
	  this->add_target(req);
	}
	else if(req.command == capabilities_.name[ik_control_capabilities::SET_HOME_TARGET])
	{
	  this->add_target(req);
	}
	else
	{
	  ROS_ERROR("IKControl::perform_ik: this is strange - you shouldn't have come this far...!");
	  return false;
	}
	used_threads_.push_back(th);
	return true;
    }
    
    return false;
}

ikControl::~ikControl()
{
    for(auto group:moveGroups_)
      delete group.second;
    
    for(int i=0; i<used_threads_.size(); i++)
      delete used_threads_.at(i);
    
    delete ik_check_;
    delete position_only_ik_check_;
    delete ik_check_legacy_;
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
  
  hand_synergy_pub_mutex_.lock();
  hand_synergy_pub_.at(hand).publish(grasp_traj);
  hand_synergy_pub_mutex_.unlock();

  return true;
}

bool ikControl::moveHand(std::string& hand, trajectory_msgs::JointTrajectory& grasp_traj)
{
  hand_synergy_pub_mutex_.lock();
  hand_synergy_pub_.at(hand).publish(grasp_traj);
  hand_synergy_pub_mutex_.unlock();
  
  return true;
}

void ikControl::simple_homing(dual_manipulation_shared::ik_service::Request req)
{
  ik_control_capabilities local_capability = ik_control_capabilities::HOME;
  // TODO: remove this!!
  end_time_mutex_.lock();
  movement_end_time_ = ros::Time(0);
  end_time_mutex_.unlock();
  
  ROS_INFO("IKControl::simple_homing: going back home...");
  std::string ee_name=req.ee_name;
  std::string group_name;
  std::vector<std::string> chain_names;
  map_mutex_.lock();
  group_name = group_map_.at(ee_name);
  if (std::find(chain_names_list_.begin(),chain_names_list_.end(),ee_name) != chain_names_list_.end())
    chain_names.push_back(ee_name);
  else
    chain_names = tree_composition_.at(ee_name);
  map_mutex_.unlock();
  
  // if the group is moving, stop it
  moveGroups_mutex_.lock();
  moveGroups_.at(ee_name)->stop();
  moveGroups_mutex_.unlock();
  // update planning_init_rs_ with current robot state
  bool target_ok = reset_robot_state(planning_init_rs_);
  // set (named) target
  target_ok = target_ok && set_target(ee_name,group_name + "_home");
  if(target_ok)
  {
    // copy robotStates
    robotState_mutex_.lock();
    moveit::core::RobotState rs_target(*target_rs_);
    moveit::core::RobotState rs_init(*planning_init_rs_);
    robotState_mutex_.unlock();
    
    // set target and start states
    moveGroups_mutex_.lock();
    target_ok = moveGroups_.at(ee_name)->setJointValueTarget(rs_target);
    if(target_ok)
      moveGroups_.at(ee_name)->setStartState(rs_init);
    moveGroups_mutex_.unlock();
  }

  dual_manipulation_shared::ik_response msg;
  msg.seq=req.seq;
  
  moveit::planning_interface::MoveGroup::Plan movePlan;

  moveit::planning_interface::MoveItErrorCode error_code(0);
  if(target_ok)
    error_code = moveGroups_.at(ee_name)->plan(movePlan);

  if(error_code.val != 1)
  {
    ROS_ERROR_STREAM("ikControl::simple_homing : unable to plan for \"" << group_name << "_home\", returning");
    msg.data = "error";
    hand_pub.at(local_capability).at(ee_name).publish(msg);
    map_mutex_.lock();
    busy.at(capabilities_.type.at(local_capability)).at(ee_name) = false;
    map_mutex_.unlock();
    // TODO: remove this!!
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now();
    end_time_mutex_.unlock();
    return;
  }
  
  moveGroups_mutex_.lock();
  error_code = moveGroups_.at(ee_name)->asyncExecute(movePlan);
  moveGroups_mutex_.unlock();
  if(error_code.val != 1)
  {
    ROS_ERROR_STREAM("ikControl::simple_homing : unable to forward \"" << group_name << "_home\" trajectory to the controller, returning");
    msg.data = "error";
    hand_pub.at(local_capability).at(ee_name).publish(msg);
    map_mutex_.lock();
    busy.at(capabilities_.type.at(local_capability)).at(ee_name) = false;
    map_mutex_.unlock();
    // TODO: remove this!!
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now();
    end_time_mutex_.unlock();
    return;
  }
  
  // also open the hand(s) we're moving home, but don't wait for it(them)
  std::vector <double > q = {0.0};
  std::vector <double > t = {1.0/hand_max_velocity};
  for(auto& ee:chain_names)
  {
    ROS_INFO_STREAM("ikControl::simple_homing : opening hand " << ee);
    moveHand(ee,q,t);
  }
  
  // a good, planned trajectory has been successfully sent to the controller
  reset_robot_state(planning_init_rs_,ee_name,movePlan.trajectory_);
  
  bool good_stop = waitForExecution(ee_name,movePlan.trajectory_);
  if(!good_stop)
  {
    msg.data = "error";
  }
  else
  {
    msg.data = "done";
  }
  hand_pub.at(local_capability).at(ee_name).publish(msg);
  map_mutex_.lock();
  busy.at(capabilities_.type.at(local_capability)).at(ee_name) = false;
  map_mutex_.unlock();
  
  return;
}

void ikControl::grasp(dual_manipulation_shared::ik_service::Request req)
{
  ik_control_capabilities local_capability = ik_control_capabilities::GRASP;
  end_time_mutex_.lock();
  movement_end_time_ = ros::Time(0);
  end_time_mutex_.unlock();
  
  ROS_INFO("IKControl::grasp: %s with %s",req.attObject.object.id.c_str(),req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;

  dual_manipulation_shared::ik_response msg;
  msg.seq=req.seq;
  
  if(grasped_obj_map_.count(req.ee_name))
  {
    ROS_ERROR_STREAM("ikControl::grasp : end-effector " << req.ee_name << " already grasped an object (obj id: " << grasped_obj_map_.at(req.ee_name) << "), returning");
    msg.data = "error";
    hand_pub.at(local_capability).at(req.ee_name).publish(msg);
    // reset movement_end_time_ in order not to block planning
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now();
    end_time_mutex_.unlock();
    return;
  }
  
  // // get timed trajectory from waypoints
  moveit_msgs::RobotTrajectory trajectory;
  moveGroups_mutex_.lock();
  moveGroups_.at(req.ee_name)->setStartState(*planning_init_rs_);
  double completed = computeTrajectoryFromWPs(trajectory,req.ee_pose,moveGroups_.at(req.ee_name),false);
  moveGroups_mutex_.unlock();
  if(completed != 1.0)
  {
    ROS_ERROR("ikControl::grasp : unable to get trajectory from waypoints, returning");
    msg.data = "error";
    hand_pub.at(local_capability).at(req.ee_name).publish(msg);
    // reset movement_end_time_ in order not to block planning
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now();
    end_time_mutex_.unlock();
    return;
  }

  // // align trajectories in time and check hand velocity limits
  computeHandTiming(trajectory,req);
  
  // // do not fill the header if you're using different computers
  
  // // execution of approach
  moveit::planning_interface::MoveGroup::Plan movePlan;
  movePlan.trajectory_ = trajectory;
  moveGroups_mutex_.lock();
  error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlan);
  moveGroups_mutex_.unlock();
  if (error_code.val != 1)
  {
    ROS_ERROR("ikControl::grasp : unable to send trajectory to the controller, returning");
    msg.data = "error";
    hand_pub.at(local_capability).at(req.ee_name).publish(msg);
    // reset movement_end_time_ in order not to block planning
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now();
    end_time_mutex_.unlock();
    return;
  }
  
#ifndef SIMPLE_GRASP
  moveHand(req.ee_name,req.grasp_trajectory);
#endif
  
  // a good, planned trajectory has been successfully sent to the controller
  reset_robot_state(planning_init_rs_,req.ee_name,trajectory);
  
  // // wait for approach
  bool good_stop = waitForExecution(req.ee_name,trajectory);
  // I didn't make it
  if (!good_stop)
  {
    ROS_ERROR("ikControl::grasp : unable to execute approach trajectory, returning");
    msg.data = "error";
    hand_pub.at(local_capability).at(req.ee_name).publish(msg);
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
    hand_pub.at(local_capability).at(req.ee_name).publish(msg);
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
  scene_object_mutex_.lock();
  scene_object_manager_.manage_object(req_obj);
  scene_object_mutex_.unlock();
  
  // we made it!
  msg.data = "done";
  hand_pub.at(local_capability).at(req.ee_name).publish(msg);
  map_mutex_.lock();
  for(auto& obj:grasped_obj_map_)
    if(obj.second == req.attObject.object.id)
    {
      grasped_obj_map_.erase(obj.first);
      break;
    }
  grasped_obj_map_[req.ee_name] = req.attObject.object.id;
  busy.at(capabilities_.type.at(local_capability)).at(req.ee_name) = false;
  map_mutex_.unlock();
  
  return;
}

void ikControl::ungrasp(dual_manipulation_shared::ik_service::Request req)
{
  ik_control_capabilities local_capability = ik_control_capabilities::UNGRASP;
  end_time_mutex_.lock();
  movement_end_time_ = ros::Time(0);
  end_time_mutex_.unlock();
  
  ROS_INFO("IKControl::ungrasp: %s from %s",req.attObject.object.id.c_str(),req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;

  dual_manipulation_shared::ik_response msg;
  msg.seq=req.seq;
  //NOTE: never check collision for waypoints (at least for now)
  bool check_collisions = false;
  
  // // get timed trajectory from waypoints
  moveit_msgs::RobotTrajectory trajectory;
  moveGroups_mutex_.lock();
  moveGroups_.at(req.ee_name)->setStartState(*planning_init_rs_);
  double completed = computeTrajectoryFromWPs(trajectory,req.ee_pose,moveGroups_.at(req.ee_name),check_collisions);
  moveGroups_mutex_.unlock();
  if(completed != 1.0)
  {
    ROS_WARN("ikControl::ungrasp : unable to get trajectory from exact waypoints, trying again with approximate ones...");
    
    bool ik_ok = true;
	
	std::string group_name;
	map_mutex_.lock();
	group_name = group_map_.at(req.ee_name);
	map_mutex_.unlock();
	
    if(!trajectory.joint_trajectory.points.empty())
      ik_ok = reset_robot_state(target_rs_,req.ee_name,trajectory);
	else
	  ik_ok = add_wp_to_traj(planning_init_rs_,group_name,trajectory);
    
    if(ik_ok)
    {
      std::vector <geometry_msgs::Pose > ee_poses;
      ee_poses.push_back(req.ee_pose.back());
      unsigned int trials_nr = 10;
      bool return_approximate_solution = true;
      //NOTE: on purpose!!! only look for a collision-free configuration!
      check_collisions = true;
      double allowed_distance = 0.5;
      
      ik_ok = set_close_target(req.ee_name,ee_poses,trials_nr,check_collisions,return_approximate_solution,allowed_distance);
      
      if(!ik_ok)
      {
	ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : set_close_target with an allowed joint-space distance of " << allowed_distance << "rads didn't work, trying again using CLIK and POSITION ONLY IK");
	bool use_clik = true;
	bool position_only_ik = true;
	ik_ok = set_target(req.ee_name,ee_poses,check_collisions,use_clik,position_only_ik);
      }
    }
    else
    {
      ROS_ERROR("ikControl::ungrasp : unable to get an IK solution for a close configuration, returning");
    }
    
    if(ik_ok)
    {
      // set last trajectory waypoint and continue
      robotState_mutex_.lock();
      ik_ok = add_wp_to_traj(target_rs_,group_name,trajectory);
      robotState_mutex_.unlock();
    }
    else
    {
      ROS_ERROR("ikControl::ungrasp : unable to add the approximate waypoint to the trajectory, returning");
    }
    
    if(!ik_ok)
    {
      // revert everything and give an error
      reset_robot_state(target_rs_);
      msg.data = "error";
      hand_pub.at(local_capability).at(req.ee_name).publish(msg);
      // reset movement_end_time_ in order not to block planning
      end_time_mutex_.lock();
      movement_end_time_ = ros::Time::now();
      end_time_mutex_.unlock();
      return;
    }
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
    hand_pub.at(local_capability).at(req.ee_name).publish(msg);
    // reset movement_end_time_ in order not to block planning
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now();
    end_time_mutex_.unlock();
    return;
  }
#endif

  if((grasped_obj_map_.count(req.ee_name) != 0) && (grasped_obj_map_.at(req.ee_name) == req.attObject.object.id))
  {
    grasped_obj_map_.erase(req.ee_name);
    
    // put the object back in the scene
    dual_manipulation_shared::scene_object_service::Request req_scene;
    req_scene.command = "detach";
    req_scene.attObject = req.attObject;
    req_scene.object_id = req.attObject.object.id;
    req_scene.object_db_id = req.object_db_id;

    scene_object_mutex_.lock();
    bool ok = scene_object_manager_.manage_object(req_scene);
    scene_object_mutex_.unlock();
    if(!ok)
    {
      ROS_WARN("IKControl::ungrasp: object with ID \"%s\" is not grasped by %s. Performing ungrasp action anyway",req.attObject.object.id.c_str(),req.ee_name.c_str());
    }
  }
  else
    ROS_WARN_STREAM("ikControl::ungrasp : end-effector " << req.ee_name << " has grasped nothing or a different object than " << req.attObject.object.id << ", not detaching it in the planning scene");

  // // execution of retreat
  moveit::planning_interface::MoveGroup::Plan movePlan;
  movePlan.trajectory_ = trajectory;
  moveGroups_mutex_.lock();
  error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlan);
  moveGroups_mutex_.unlock();
  if (error_code.val != 1)
  {
    ROS_ERROR("ikControl::ungrasp : unable to send trajectory to the controller, returning");
    msg.data = "error";
    hand_pub.at(local_capability).at(req.ee_name).publish(msg);
    // reset movement_end_time_ in order not to block planning
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now();
    end_time_mutex_.unlock();
    return;
  }
  
  // a good, planned trajectory has been successfully sent to the controller
  reset_robot_state(planning_init_rs_,req.ee_name,trajectory);
  
  // // wait for retreat
  good_stop = waitForExecution(req.ee_name,trajectory);
  // I didn't make it
  if (!good_stop)
  {
    ROS_ERROR("ikControl::ungrasp : unable to execute retreat trajectory, returning");
    msg.data = "error";
    hand_pub.at(local_capability).at(req.ee_name).publish(msg);
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
    hand_pub.at(local_capability).at(req.ee_name).publish(msg);
    return;
  }
#endif

  msg.data = "done";
  hand_pub.at(local_capability).at(req.ee_name).publish(msg);
  map_mutex_.lock();
  busy.at(capabilities_.type.at(local_capability)).at(req.ee_name) = false;
  map_mutex_.unlock();
  
  return;
}

bool ikControl::reset_robot_state(const moveit::core::RobotStatePtr& rs)
{
  moveGroups_mutex_.lock();
  moveit::core::RobotState kinematic_state(*(moveGroups_.begin()->second->getCurrentState()));
  moveGroups_mutex_.unlock();
  
  std::unique_lock<std::mutex>(robotState_mutex_);

  ROS_INFO_STREAM("ikControl::reset_robot_state : resetting " << rs->getRobotModel()->getName());
  
  // minimal checks - are more checks needed?
  assert(kinematic_state.getVariableCount() == rs->getVariableCount());
  assert(kinematic_state.getRobotModel()->getName() == rs->getRobotModel()->getName());
  
  for(int i=0; i<rs->getVariableCount(); i++)
    rs->setVariablePosition(i,kinematic_state.getVariablePosition(i));
  
  return true;
}

bool ikControl::reset_robot_state(const moveit::core::RobotStatePtr& rs, std::string ee_name, const moveit_msgs::RobotTrajectory& traj)
{
  std::string group_name;
  map_mutex_.lock();
  group_name = group_map_.at(ee_name);
  map_mutex_.unlock();
  
  std::unique_lock<std::mutex>(robotState_mutex_);

  ROS_INFO_STREAM("ikControl::reset_robot_state : resetting " << rs->getRobotModel()->getName() << " with a trajectory for " << ee_name);

  //NOTE: robot_traj, built on robot_model, contains the full robot; trajectory, instead, is only for the group joints
  robot_trajectory::RobotTrajectory robot_traj(rs->getRobotModel(),rs->getJointModelGroup(group_name)->getName());
  robot_traj.setRobotTrajectoryMsg(*rs,traj);
  
  // minimal checks - are more checks needed?
  assert(robot_traj.getLastWayPoint().getVariableCount() == rs->getVariableCount());
  assert(robot_traj.getLastWayPoint().getRobotModel()->getName() == rs->getRobotModel()->getName());
  
  for(int i=0; i<rs->getVariableCount(); i++)
    rs->setVariablePosition(i,robot_traj.getLastWayPoint().getVariablePosition(i));
  
  return true;
}

bool ikControl::set_target(std::string ee_name, std::string named_target)
{
  std::string group_name;
  map_mutex_.lock();
  group_name = group_map_.at(ee_name);
  map_mutex_.unlock();
  
  std::unique_lock<std::mutex>(robotState_mutex_);
  
  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
  
  bool set_ok = target_rs_->setToDefaultValues(jmg,named_target);
  
  set_ok = set_ok && ik_check_->reset_robot_state(*target_rs_);
  
  return set_ok;
}

bool ikControl::set_target(std::string ee_name, std::vector< geometry_msgs::Pose > ee_poses, bool check_collisions, bool use_clik, bool position_only)
{
  std::unique_lock<std::mutex>(robotState_mutex_);
  
  // give an initial guess to ik_check_
  if(!ik_check_->reset_robot_state(*target_rs_))
  {
    ROS_ERROR_STREAM("ikControl::set_target : unable to reset ik_check");
    return false;
  }
  
  std::vector<std::vector<double>> solutions;
  const std::vector<double> initial_guess = std::vector<double>();
  bool ik_ok;
  ik_ok = ik_check_->find_group_ik(ee_name,ee_poses,solutions,initial_guess,check_collisions);
  if(!ik_ok && use_clik)
  {
    double clik_res = ik_check_->clik(ee_name,ee_poses,solutions,initial_guess,check_collisions);
    ik_ok = clik_res > clik_threshold_;
    if(!ik_ok)
      ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : found CLIK solution up to " << 100.0*clik_res << "% of the initial gap, below the allowed threshold of " << clik_threshold_*100.0 << "%");
  }
  if(!ik_ok && position_only)
  {
    if(!position_only_ik_check_->reset_robot_state(ik_check_->get_robot_state()))
      ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : unable to reset position_only_ik_check_");
    
    ik_ok = position_only_ik_check_->find_group_ik(ee_name,ee_poses,solutions,initial_guess,check_collisions);
    if(!ik_ok && use_clik)
    {
      double clik_res = position_only_ik_check_->clik(ee_name,ee_poses,solutions,initial_guess,check_collisions);
      ik_ok = clik_res > clik_threshold_;
      if(!ik_ok)
	ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : found POSITION ONLY CLIK solution up to " << 100.0*clik_res << "% of the initial gap, below the allowed threshold of " << clik_threshold_*100.0 << "%");
    }
    
    if(ik_ok && !ik_check_->reset_robot_state(position_only_ik_check_->get_robot_state()))
    {
      ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : unable to reset ik_check");
      return false;
    }
  }
  
  if(!ik_ok)
  {
    ROS_ERROR_STREAM("ikControl::set_target : unable to find " << (position_only?"position_only ":"") << "IK for the requested pose " << (check_collisions?"":"NOT ") << "checking collisions and " << (use_clik?"":"NOT ") << "using CLIK");
    // this is if using the target before calling this function again
    ik_check_->reset_robot_state(*target_rs_);
    return false;
  }
  
  // update target_rs_ for next time this function will be called
  for(int i=0; i<ik_check_->get_robot_state().getVariableCount(); i++)
    target_rs_->setVariablePosition(i,ik_check_->get_robot_state().getVariablePosition(i));
  
  return true;
}

bool ikControl::set_close_target(std::string ee_name, std::vector< geometry_msgs::Pose > ee_poses, unsigned int trials_nr, bool check_collisions, bool return_approximate_solution, double allowed_distance)
{
  std::string group_name;
  std::vector<std::string> chain_names;
  map_mutex_.lock();
  group_name = group_map_.at(ee_name);
  if (std::find(chain_names_list_.begin(),chain_names_list_.end(),ee_name) != chain_names_list_.end())
    chain_names.push_back(ee_name);
  else
    chain_names = tree_composition_.at(ee_name);
  map_mutex_.unlock();
  
  std::unique_lock<std::mutex>(robotState_mutex_);
  
  // give an initial guess to ik_check_
  if(!ik_check_->reset_robot_state(*target_rs_))
  {
    ROS_ERROR_STREAM("ikControl::set_target : unable to reset ik_check");
    return false;
  }
  
  std::vector<std::vector<double>> solutions;
  std::vector <ik_iteration_info> it_info;
  bool store_iterations = false;
  std::vector<double> initial_guess;

  bool ik_ok = ik_check_->find_closest_group_ik(ee_name,ee_poses,solutions,it_info,store_iterations,allowed_distance,trials_nr,initial_guess,check_collisions,return_approximate_solution);
  
  if(solutions.empty() || !ik_ok)
  {
    ROS_ERROR_STREAM("ikControl::set_target : unable to find IK for the requested pose");
    // this is if using the target before calling this function again
    ik_check_->reset_robot_state(*target_rs_);
    return false;
  }
  
  for(int i=0; i<chain_names.size(); i++)
    ik_check_->reset_robot_state(chain_names.at(i),solutions.at(i));
  
  // update target_rs_ for next time this function will be called
  for(int i=0; i<ik_check_->get_robot_state().getVariableCount(); i++)
    target_rs_->setVariablePosition(i,ik_check_->get_robot_state().getVariablePosition(i));
  
  return true;
}

void ikControl::add_target(const dual_manipulation_shared::ik_service::Request& req)
{
  std::unique_lock<std::mutex>(map_mutex_);
  ik_control_capabilities local_capability = capabilities_.from_name[req.command];
  
  // if it's a tree, clear all previously set targets for its chains
  if (std::find(tree_names_list_.begin(),tree_names_list_.end(),req.ee_name) != tree_names_list_.end())
  {
    for(auto chain:tree_composition_.at(req.ee_name))
      targets_.erase(chain);
  }
  // else, if it's a chain, split any previously set target for the whole tree into chain targets
  else
  {
    for(auto tree:tree_names_list_)
      if(std::find(tree_composition_.at(tree).begin(),tree_composition_.at(tree).end(),req.ee_name) != tree_composition_.at(tree).end())
      {
	if (targets_.count(tree) == 0)
	  continue;
	
	if(targets_[tree].type == ik_target_type::POSE_TARGET)
	{
	  int i=0;
	  for(auto chain:tree_composition_.at(tree))
	    targets_[chain] = ik_target(targets_[tree].ee_poses.at(i++),chain);
	  targets_.erase(tree);
	}
	else if(targets_[tree].type == ik_target_type::JOINT_TARGET)
	{
	  int i=0;
	  for(auto chain:tree_composition_.at(tree))
	    targets_[chain] = ik_target(targets_[tree].joints.at(i++),chain);
	  targets_.erase(tree);
	}
	else if(targets_[tree].type == ik_target_type::NAMED_TARGET)
	{
	  std::string suffix = targets_[tree].target_name;
	  //NOTE: this hp is that each target is named with the same suffix for each chain/tree, starting with the chain/tree name
	  suffix = suffix.substr(tree.size(),suffix.size());
	  for(auto chain:tree_composition_.at(tree))
	    targets_[chain] = ik_target(chain + suffix,chain);
	  targets_.erase(tree);
	}
	else
	  ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : Unknown ik_target_type!!!");
      }
  }
  
  if(local_capability == ik_control_capabilities::SET_TARGET)
  {
    targets_[req.ee_name] = ik_target(req.ee_pose,req.ee_name);
  }
  else if(local_capability == ik_control_capabilities::SET_HOME_TARGET)
  {
    targets_[req.ee_name] = ik_target(group_map_.at(req.ee_name) + "_home",req.ee_name);
  }
  else
    ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : requested set-target command \'" << req.command << "\' is not implemented!");
  
  busy.at(capabilities_.type.at(local_capability)).at(req.ee_name) = false;
}
