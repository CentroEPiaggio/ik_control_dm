#include "ik_control.h"
#include "trajectory_utils.h"
#include <dual_manipulation_shared/parsing_utils.h>
#include <dual_manipulation_shared/ik_response.h>
#include "moveit/trajectory_execution_manager/trajectory_execution_manager.h"
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>
//#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf_conversions/tf_kdl.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/String.h>
#include <ros/console.h>

#define SIMPLE_GRASP 1
#define CLASS_NAMESPACE "ikControl::"
#define CLASS_LOGNAME "ikControl"
#define DEFAULT_MAX_PLANNING_ATTEMPTS 1
#define HIGH_UNGRASP_WP_IF_COLLIDING 0.1
#define MOTION_PLAN_REQUEST_TESTING 1
#define MOTION_PLAN_REQUEST_TESTING_ADVANCED 1
#define TABLE_WP_HEIGHT 0.1
#define DEBUG 0
#define MAX_REPLAN 10
#define ALLOWED_JOINT_JUMP 0.5 // allow at most ALLOWED_JOINT_JUMP rads jump per joint between two successive points in a trajectory
#define CLOSED_HAND 1.0

using namespace dual_manipulation::ik_control;

ikControl::ikControl()
{
    if( ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Warn) & ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) )
        ros::console::notifyLoggerLevelsChanged();
    // // to access a named logger (e.g. the one named with #define CLASS_LOGNAME) use the following syntax
    // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME "." + CLASS_LOGNAME, ros::console::levels::Warn) ) {
    
    setDefaultParameters();
    
    if (node.getParam("ik_control_parameters", ik_control_params))
      parseParameters(ik_control_params);
    
    setParameterDependentVariables();
}

void ikControl::reset()
{ 
  robotState_mutex_.lock();
  reset_robot_state(planning_init_rs_); reset_robot_state(target_rs_);
  robotState_mutex_.unlock();
  movePlans_mutex_.lock();
  for(auto& plan:movePlans_){ move_group_interface::MoveGroup::Plan tmp_plan; std::swap(plan.second,tmp_plan);}
  movePlans_mutex_.unlock();
  end_time_mutex_.lock();
  movement_end_time_ = ros::Time::now();
  end_time_mutex_.unlock();
  
  map_mutex_.lock();
  targets_.clear();
  grasped_obj_map_.clear();
  objects_map_.clear();
  map_mutex_.unlock();
  // for the first time, update the planning scene in full
  moveit_msgs::GetPlanningScene srv;
  uint32_t objects = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
  srv.request.components.components = objects;
  if(!scene_client_.call(srv))
    ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to call /get_planning_scene service - starting with an empty planning scene...");
  else
  {
    for(auto attObject:srv.response.scene.robot_state.attached_collision_objects)
    {
      moveit_msgs::AttachedCollisionObject obj;
      obj.link_name;
      for(auto links:allowed_collisions_)
	if(std::find(links.second.begin(),links.second.end(),attObject.link_name)!=links.second.end())
	{
	  grasped_obj_map_[links.first] = attObject.object.id;
	  objects_map_[attObject.object.id] = attObject;
	}
    }
  }
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
    backup_max_planning_attempts_ = DEFAULT_MAX_PLANNING_ATTEMPTS;
    goal_position_tolerance_ = 0.005;
    goal_orientation_tolerance_ = 0.005;
    goal_joint_tolerance_ = 0.005;
    ws_bounds_.assign({-1.2,-1.5,0.1,0.2,1.5,1.5});
    
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
    
    trajectory_event_publisher_ = node.advertise<std_msgs::String>(trajectory_execution_manager::TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC, 1, false);
    scene_client_ = node.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    motionPlan_client_ = node.serviceClient<moveit_msgs::GetMotionPlan>(move_group::PLANNER_SERVICE_NAME);

    allowed_excursions_["left_hand"].clear();
    allowed_excursions_["left_hand"].assign({0.5,0.5,1.0,1.0,6.0,6.0,6.0});
    allowed_excursions_["right_hand"].clear();
    allowed_excursions_["right_hand"].assign({0.5,0.5,1.0,1.0,6.0,6.0,6.0});
    allowed_excursions_["both_hands"].clear();
    allowed_excursions_["both_hands"].assign({0.5,0.5,1.0,1.0,6.0,6.0,6.0,0.5,0.5,1.0,1.0,6.0,6.0,6.0});
    
    // apart from the first time, when this is done in the constructor after parameters are obtained from the server
    if(moveGroups_.size() > 0)
    {
      for(auto group:moveGroups_)
	delete group.second;
      moveGroups_.clear();
      movePlans_.clear();
      busy.clear();
      hand_pub.clear();
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
      busy[capabilities_.type[capability.first]][group_name.first] = false;
    }
  }
  
  for(auto capability:capabilities_.name)
  {
    hand_pub[capability.first] = node.advertise<dual_manipulation_shared::ik_response>("/ik_control/" + capabilities_.msg[capability.first],1,this);
    ROS_DEBUG_STREAM("hand_pub[" << capability.second << "] => /ik_control/" + capabilities_.msg[capability.first]);
  }
  
  for(auto item:moveGroups_)
  {
    item.second->setPlannerId(backup_planner_id_);
    item.second->setPlanningTime(backup_planning_time_);
    item.second->setNumPlanningAttempts(backup_max_planning_attempts_);
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
      for (auto acpref:allowed_collision_prefixes_[chain_name])
	if(link.compare(0,acpref.size(),acpref.c_str()) == 0)
	{
	  allowed_collisions_[chain_name].push_back(link);
	  break;
	}

    // JointTrajectory publishers
    if(hand_synergy_pub_topics_.count(chain_name))
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
  visual_rs_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_));
  
  // default constructor will read values from the ROS parameter server, which are loaded once we load move_group (see "omp_planning_pipeline.launch.xml")
  ros::NodeHandle move_group_node("move_group");
  ros::NodeHandle private_nh("~");
  if(node.hasParam("ik_control_parameters/fix_start_state_collision/jiggle_fraction"))
  {
    double jiggle_fraction;
    node.getParam("ik_control_parameters/fix_start_state_collision/jiggle_fraction",jiggle_fraction);
    private_nh.setParam("jiggle_fraction",jiggle_fraction);
  }
  if(node.hasParam("ik_control_parameters/fix_start_state_collision/max_sampling_attempts"))
  {
    double max_sampling_attempts;
    node.getParam("ik_control_parameters/fix_start_state_collision/max_sampling_attempts",max_sampling_attempts);
    private_nh.setParam("max_sampling_attempts",max_sampling_attempts);
  }
  robotState_mutex_.lock();
  pipeline_ = planning_pipeline::PlanningPipelinePtr(new planning_pipeline::PlanningPipeline(target_rs_->getRobotModel(),move_group_node,"planning_plugin","request_adapters"));
  robotState_mutex_.unlock();
  
  MotionPlanReq_.allowed_planning_time = planning_time_;
  MotionPlanReq_.num_planning_attempts = max_planning_attempts_;
  MotionPlanReq_.planner_id = planner_id_;
  MotionPlanReq_.workspace_parameters.header.frame_id = robot_model_->getRootLinkName();
  geometry_msgs::Vector3 min_corner,max_corner;
  min_corner.x = ws_bounds_.at(0); min_corner.y = ws_bounds_.at(1); min_corner.z = ws_bounds_.at(2);
  max_corner.x = ws_bounds_.at(3); max_corner.y = ws_bounds_.at(4); max_corner.z = ws_bounds_.at(5);
  MotionPlanReq_.workspace_parameters.min_corner = min_corner;
  MotionPlanReq_.workspace_parameters.max_corner = max_corner;
  
  ikCheck_mutex_.lock();
  // TODO: check whether we need updated or not... I would say yes, and not including the AttachedCollisionObject in the robot state when wanting no collision checking
  planning_scene_ = ik_check_->get_planning_scene(true);
  ikCheck_mutex_.unlock();
  
  reset();
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
	ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : No composition is specified for tree '" << tree << "': check the yaml configuration.");
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
      parseSingleParameter(params["motion_planner"],backup_max_planning_attempts_,"backup_max_planning_attempts");
      if(max_planning_attempts_ <= 0) max_planning_attempts_ = DEFAULT_MAX_PLANNING_ATTEMPTS;
      if(backup_max_planning_attempts_ <= 0) backup_max_planning_attempts_ = DEFAULT_MAX_PLANNING_ATTEMPTS;
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

bool ikControl::waitForHandMoved(std::string& hand, double hand_target, const trajectory_msgs::JointTrajectory& traj)
{
  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : entered");
  
  // if an end-effector is not a hand
  if(!hand_actuated_joint_.count(hand))
      return true;
  
  if(kinematics_only_)
  {
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : kinematics_only execution - moving on after the trajectory has been shown");
    moveit_msgs::RobotTrajectory traj2;
    traj2.joint_trajectory = traj;
    publishTrajectoryPath(traj2);
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
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : " << hand_actuated_joint_.at(hand) << " NOT found in /joint_states - returning");
    return false;
  }

  // wait for up to 10 more seconds
  while(counter<100)
  {
    //get joint states
    joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",node,ros::Duration(3));
    
    if(joint_states->name.at(hand_index) != hand_actuated_joint_.at(hand))
    {
      ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : joints in joint_states changed order");
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
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : exiting with good_stop OK");
  else
    ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : exiting with error");
  return good_stop;
}

bool ikControl::waitForExecution(std::string ee_name, moveit_msgs::RobotTrajectory traj)
{
  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : entered");
  
  if (traj.joint_trajectory.points.size() == 0)
  {
    ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the trajectory to wait for was empty");
    return true;
  }
  
  if(kinematics_only_)
  {
    publishTrajectoryPath(traj);
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : kinematics_only execution - moving on after the trajectory has been shown");
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now() + traj.joint_trajectory.points.back().time_from_start;
    end_time_mutex_.unlock();
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
    timeout = timeout*1.0;
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : waiting for at most " << timeout << " (trajectory total time)");
    pt = ros::topic::waitForMessage<control_msgs::FollowJointTrajectoryActionResult>(controller_name + "result",node,timeout);
    if(pt)
      ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : received message - error_code=" << pt->result.error_code);
    else
      ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : timeout reached");
  }
  else
  {
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : waiting for at least " << timeout << " (trajectory total time)");
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
	ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : couldn't find requested joints!");
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
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : vel=" << vel << " (< " << velocity_threshold << ") but dist=" << dist << " (>= " << position_threshold << ")");
	break;
      }
    }
    usleep(100000);
  }
  
  if(good_stop)
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : exiting with good_stop OK");
  else
  {
    ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : exiting with error");
    reset();
  }
  return good_stop;
}

void ikControl::ik_check_thread(dual_manipulation_shared::ik_service::Request req)
{
  ik_control_capabilities local_capability = ik_control_capabilities::IK_CHECK;
  
  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Thread spawned! Computing IK for " << req.ee_name);

  dual_manipulation_shared::ik_response msg;
  msg.seq=req.seq;
  msg.group_name = req.ee_name;
  
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
  hand_pub.at(local_capability).publish(msg); //publish on a topic when the IK check is done

  map_mutex_.lock();
  busy.at(capabilities_.type.at(local_capability)).at(req.ee_name)=false;
  map_mutex_.unlock();
  
  return;
}

bool ikControl::build_motionPlan_request(moveit_msgs::MotionPlanRequest& req, const std::map<std::string,ik_target>& targets, ik_control_capabilities plan_type)
{
  // TODO: define a set of tolerances depending on the capability (these will be parameterized from outside...!)
  std::map<ik_control_capabilities,double> position_tolerance;
  std::map<ik_control_capabilities,double> orientation_tolerance;
  
  position_tolerance[ik_control_capabilities::PLAN] = goal_position_tolerance_;
  position_tolerance[ik_control_capabilities::PLAN_BEST_EFFORT] = 10*goal_position_tolerance_;
  position_tolerance[ik_control_capabilities::PLAN_NO_COLLISION] = goal_position_tolerance_;
  position_tolerance[ik_control_capabilities::PLAN_CLOSE_BEST_EFFORT] = 10*goal_position_tolerance_;
  orientation_tolerance[ik_control_capabilities::PLAN] = goal_orientation_tolerance_;
  orientation_tolerance[ik_control_capabilities::PLAN_BEST_EFFORT] = 5*goal_orientation_tolerance_;
  orientation_tolerance[ik_control_capabilities::PLAN_NO_COLLISION] = goal_orientation_tolerance_;
  orientation_tolerance[ik_control_capabilities::PLAN_CLOSE_BEST_EFFORT] = 50*goal_orientation_tolerance_;
  
  if(!position_tolerance.count(plan_type))
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unknown plan_type!!! returning...");
    return false;
  }
  
  double pos_tol = position_tolerance.at(plan_type);
  double orient_tol = orientation_tolerance.at(plan_type);
  double joint_tol = goal_joint_tolerance_;
  
  bool position_only = (plan_type == ik_control_capabilities::PLAN_BEST_EFFORT || plan_type == ik_control_capabilities::PLAN_CLOSE_BEST_EFFORT);
  if(position_only)
  {
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : planning position_only > increasing the position tolerance from " << goal_position_tolerance_ << " to " << pos_tol);
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : planning position_only > increasing the orientation tolerance from " << goal_orientation_tolerance_ << " to " << orient_tol);
  }
  bool is_close = (plan_type == ik_control_capabilities::PLAN_NO_COLLISION || plan_type == ik_control_capabilities::PLAN_CLOSE_BEST_EFFORT);
  
  moveit_msgs::Constraints c;
  
  for(auto target_it:targets)
  {
    ik_target& target(target_it.second);
    moveit_msgs::Constraints c_tmp;
    
    std::string group_name;
    map_mutex_.lock();
    group_name = group_map_.at(target.ee_name);
    map_mutex_.unlock();
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
    
    if(target.type == ik_target_type::NAMED_TARGET)
    {
      set_target(target.ee_name,target.target_name);
      
      c_tmp = kinematic_constraints::constructGoalConstraints(*target_rs_,jmg,joint_tol);
    }
    else if(target.type == ik_target_type::POSE_TARGET)
    {
      moveit_msgs::Constraints c_tmp2;
      
      // NOTE: terrible way of getting end-effector names...
      std::vector<std::string> tips;
      if(jmg->isEndEffector())
	tips.emplace_back(jmg->getEndEffectorParentGroup().second);
      else
	jmg->getEndEffectorTips(tips);
      
      // let's make sure we're getting the right coupling...
      assert(tips.size() == target.ee_poses.size());
      
      // go through the poses...
      for(int i=0; i<target.ee_poses.size(); i++)
      {
	// if(position_only)
	// {
	//   geometry_msgs::PointStamped point;
	//   point.header.frame_id = robot_model_->getRootLinkName();
	//   point.point = target.ee_poses.at(i).position;
	//   c_tmp2 = kinematic_constraints::constructGoalConstraints(tips.at(i),point,pos_tol);
	// }
	// else
	// {
	  geometry_msgs::PoseStamped pose;
	  geometry_msgs::Pose normalized_pose = target.ee_poses.at(i);
	  double norm = std::sqrt( normalized_pose.orientation.x*normalized_pose.orientation.x + normalized_pose.orientation.y*normalized_pose.orientation.y + normalized_pose.orientation.z*normalized_pose.orientation.z + normalized_pose.orientation.w*normalized_pose.orientation.w );
	  normalized_pose.orientation.x = normalized_pose.orientation.x/norm;
	  normalized_pose.orientation.y = normalized_pose.orientation.y/norm;
	  normalized_pose.orientation.z = normalized_pose.orientation.z/norm;
	  normalized_pose.orientation.w = normalized_pose.orientation.w/norm;
      if(norm < 0.99 || norm > 1.01)
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Setting target pose (Q norm = " << norm << ") : " << target.ee_poses.at(i) << "normalized became: " << normalized_pose);
	  pose.header.frame_id = robot_model_->getRootLinkName();
	  pose.pose = normalized_pose;
	  c_tmp2 = kinematic_constraints::constructGoalConstraints(tips.at(i),pose,pos_tol,orient_tol);
	// }
	c_tmp = kinematic_constraints::mergeConstraints(c_tmp,c_tmp2);
      }
    }
    else // if(target.type == ik_target_type::JOINT_TARGET)
    {
      ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the requested target type is NOT implemented yet!!!");
      return false;
    }
    c = kinematic_constraints::mergeConstraints(c,c_tmp);
  }
  
  // merge everything into the request
  if(MotionPlanReq_.goal_constraints.empty())
    MotionPlanReq_.goal_constraints.push_back(c);
  else
    MotionPlanReq_.goal_constraints.at(0) = kinematic_constraints::mergeConstraints(MotionPlanReq_.goal_constraints.at(0),c);
  
  if(MotionPlanReq_.goal_constraints.size() > 1)
    ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : multiple goals are set, but current implementation of this software only considers first! Ignoring the others...");
  
  if(is_close)
  {
#if DEBUG>1
    ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : planning to a close configuration > implement-me better! I am assuming there is only ONE POSE TARGET here, AND that table waypoints are " << TABLE_WP_HEIGHT*100 << "cm high!");
    
    std::cout << c << std::endl;
#endif
    
    moveit_msgs::PositionConstraint pc;
    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.push_back(1.5); // BOX_X
    box.dimensions.push_back(1.5); // BOX_Y
    // compute BOX_Z such that it doubles the distance from the initial point
    // TODO: compute this!!! at now assuming table waypoints are always 10cm high!
    // allow for more space to plan within
    box.dimensions.push_back(10.0*TABLE_WP_HEIGHT); // was (2.1*TABLE_WP_HEIGHT);
    
    pc = c.position_constraints.at(0);
    pc.constraint_region.primitives.clear();
    pc.constraint_region.primitives.push_back(box);
    // the orientation of the goal constraint is already [0 0 0 1]
    if(plan_type == ik_control_capabilities::PLAN_NO_COLLISION)
      pc.constraint_region.primitive_poses.at(0).position.z += TABLE_WP_HEIGHT;
    
    req.path_constraints.name = "my_box_constraint";
    req.path_constraints.position_constraints.clear();
    req.path_constraints.position_constraints.push_back(pc);
    
#if DEBUG > 1
    ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : planning to a close configuration > implement-me better! I am assuming there is only ONE POSE TARGET here!");
    std::cout << "Path constraint:\n" << req.path_constraints.position_constraints.at(0) << std::endl;
#endif
  }
  
  return true;

//   // THIS is managed outside!
//   req.start_state.attached_collision_objects
  
//   req.path_constraints
//   req.trajectory_constraints
}

void ikControl::planning_thread(dual_manipulation_shared::ik_service::Request req, bool check_collisions, bool use_clik, bool is_close)
{
    ik_control_capabilities local_capability;
    if(!check_collisions && !use_clik && is_close)
      local_capability = ik_control_capabilities::PLAN_NO_COLLISION;
    else if(check_collisions && !use_clik && !is_close)
      local_capability = ik_control_capabilities::PLAN;
    else if(check_collisions && use_clik && !is_close)
      local_capability = ik_control_capabilities::PLAN_BEST_EFFORT;
    else if(check_collisions && use_clik && is_close)
      local_capability = ik_control_capabilities::PLAN_CLOSE_BEST_EFFORT;
    else
      ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the requested capability is NOT implemented yet!!!");
  
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Thread spawned! Computing plan for " << req.ee_name);
    
    move_group_interface::MoveGroup* localMoveGroup;
    std::string group_name;
    std::string group_name_true;
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
      ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : no target needs to be set...");
    }
    map_mutex_.unlock();
    
    if(group_name == "full_robot")
    {
        std::vector<std::string> targets;
        for(auto& t:local_targets)
            targets.push_back(t.first);
        group_name_true = findGroupName(targets);
    }
    else
        group_name_true = group_name;
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Planning for group " << group_name << " (" << group_name_true << ")");
//     geometry_msgs::Pose current_pose = localMoveGroup->getCurrentPose().pose;
//     
//     std::cout << "pos [x y z]: " << current_pose.position.x << " " << current_pose.position.y << " " << current_pose.position.z << std::endl;
//     std::cout << "orient [x y z w]: "  << current_pose.orientation.x << " " << current_pose.orientation.y << " " << current_pose.orientation.z << " " << current_pose.orientation.w << std::endl;

    moveit::planning_interface::MoveItErrorCode error_code;

    dual_manipulation_shared::ik_response msg;
    msg.seq=req.seq;
    msg.group_name = req.ee_name;
    moveit::planning_interface::MoveGroup::Plan movePlan;

#if !MOTION_PLAN_REQUEST_TESTING_ADVANCED
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
	ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : joint-value targets are not supported yet! Ignoring the target set for " << target.ee_name);
	// target_set = target_set && set_target(target.ee_name,target.joints);
    }
    for(auto target_p:local_targets)
    {
      ik_target& target(target_p.second);
      if(target.type == ik_target_type::POSE_TARGET)
	target_set = target_set && set_target(target.ee_name,target.ee_poses,check_collisions,use_clik,position_only_ik,is_close);
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
      ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Target set correctly!" << std::endl);
    }
    else
    {
      ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unable to set target pose\n");
      msg.data = "error";
      hand_pub.at(local_capability).publish(msg); //publish on a topic when the trajectory is done

      map_mutex_.lock();
      busy.at(capabilities_.type.at(local_capability)).at(req.ee_name)=false;
      map_mutex_.unlock();

      return;
    }
    
    // if I'm planning a long trajectory, use MoveIt!, else just add waypoints...
    if(!is_close)
#else
    // add a check for generated plans: if the jump is too high, try replanning
    bool need_replan = true;
    int count = 0;
    while (need_replan && (count++ < MAX_REPLAN))
#endif
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
      ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : will use " << plan_time << "s planning time, max between default " << planning_time_ << "s and residual movemenent time " << residual_move_time.toSec() << "s");

#if MOTION_PLAN_REQUEST_TESTING
    
    planning_interface::MotionPlanResponse MotionPlanRes;
    
    MotionPlanReq_.allowed_planning_time = plan_time;
    MotionPlanReq_.group_name = group_name_true;
    MotionPlanReq_.num_planning_attempts = max_planning_attempts_;
    MotionPlanReq_.planner_id = planner_id_;
    
    bool copy_attached_bodies(check_collisions);
    
    robotState_mutex_.lock();
    // TODO: add here the object..?! use updated planning scene and robot state as NOT DIFF...
    moveit::core::robotStateToRobotStateMsg(*planning_init_rs_,MotionPlanReq_.start_state);
    robotState_mutex_.unlock();
    MotionPlanReq_.start_state.attached_collision_objects.clear();
    if(copy_attached_bodies)
    {
      map_mutex_.lock();
      for(auto attObject:objects_map_)
	MotionPlanReq_.start_state.attached_collision_objects.push_back(attObject.second);
      map_mutex_.unlock();
    }
#if DEBUG>1
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : debugging attachedn collision objects...\n");
    for(auto attObject:MotionPlanReq_.start_state.attached_collision_objects)
    {
      std::cout << attObject.object.id << ": touch links are > | " << std::endl;
      for(auto link:attObject.touch_links)
	std::cout << link << " | ";
      std::cout << std::endl;
    }
    std::cout << "waiting for input..." << std::endl;;
    char y;
    std::cin >> y;
#endif
    MotionPlanReq_.start_state.is_diff = false;

    //ATTENTION: here doubling code on purpose, this will go away if we decide to keep this version and merge everything together
    MotionPlanReq_.goal_constraints.clear();
    moveit_msgs::Constraints empty_constr;
    MotionPlanReq_.path_constraints = empty_constr;
    moveit_msgs::TrajectoryConstraints empty_traj_constr;
    MotionPlanReq_.trajectory_constraints = empty_traj_constr;
    bool mp_req = build_motionPlan_request(MotionPlanReq_,local_targets,local_capability);
    
    if(!mp_req)
      ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to obtain motion planning request!!!");
    else if(pipeline_->generatePlan(planning_scene_,MotionPlanReq_,MotionPlanRes))
    {
      moveit_msgs::MotionPlanResponse msg;
      MotionPlanRes.getMessage(msg);
      movePlan.trajectory_ = msg.trajectory;
    }
    error_code = MotionPlanRes.error_code_;
    
#else
    
      localMoveGroup->setPlanningTime(plan_time);
      error_code = localMoveGroup->plan(movePlan);
      
#endif
      
      if(error_code.val != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
	ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to plan with \'" << planner_id_ << "\' with timeout of " << plan_time << "s, trying once more with \'" << backup_planner_id_ << "\' and timeout of " << planning_time_ << "s");
	// attempt a chance planning with a random algorithm and a longer waiting: if it fails, it should fail

#if MOTION_PLAN_REQUEST_TESTING
	
	MotionPlanReq_.planner_id = backup_planner_id_;
	MotionPlanReq_.allowed_planning_time = planning_time_;
	MotionPlanReq_.num_planning_attempts = backup_max_planning_attempts_;
	if(pipeline_->generatePlan(planning_scene_,MotionPlanReq_,MotionPlanRes))
	{
	  moveit_msgs::MotionPlanResponse msg;
	  MotionPlanRes.getMessage(msg);
	  movePlan.trajectory_ = msg.trajectory;
	}
	error_code = MotionPlanRes.error_code_;
	
	// 2nd level of backup planning...
	if(error_code.val != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
	  ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to plan with \'" << backup_planner_id_ << "\' with timeout of " << plan_time << "s, trying last time with \'" << backup_planner_id_ << "\' and timeout of " << backup_planning_time_ << "s");

	  MotionPlanReq_.planner_id = backup_planner_id_;
	  MotionPlanReq_.allowed_planning_time = backup_planning_time_;
	  MotionPlanReq_.num_planning_attempts = backup_max_planning_attempts_;
	  if(pipeline_->generatePlan(planning_scene_,MotionPlanReq_,MotionPlanRes))
	  {
	    moveit_msgs::MotionPlanResponse msg;
	    MotionPlanRes.getMessage(msg);
	    movePlan.trajectory_ = msg.trajectory;
	  }
	  error_code = MotionPlanRes.error_code_;
	}
	MotionPlanReq_.planner_id = planner_id_;
	MotionPlanReq_.allowed_planning_time = planning_time_;
	MotionPlanReq_.num_planning_attempts = max_planning_attempts_;
	
#else
	localMoveGroup->setPlannerId(backup_planner_id_);
	localMoveGroup->setPlanningTime(backup_planning_time_);
	error_code = localMoveGroup->plan(movePlan);
	// reset the planner ID
	localMoveGroup->setPlannerId(planner_id_);
#endif
      }
#if MOTION_PLAN_REQUEST_TESTING_ADVANCED
      if(!movePlan.trajectory_.joint_trajectory.points.empty())
        need_replan = !check_trajectory_continuity(movePlan.trajectory_,ALLOWED_JOINT_JUMP);
      else
        need_replan = false;
      // make sure any wrong plan is erased
      if(need_replan)
          movePlan.trajectory_ = moveit_msgs::RobotTrajectory();
      //TODO: instead of check and replan, it would be possible to try enforcing the bound... remember this would need a time reparametrization!
#endif
    }
#if !MOTION_PLAN_REQUEST_TESTING_ADVANCED
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
#endif
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : movePlan traj size: " << movePlan.trajectory_.joint_trajectory.points.size() << std::endl);
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
      
#if MOTION_PLAN_REQUEST_TESTING_ADVANCED
      // hope these help with visualization...
      robotState_mutex_.lock();
      moveit::core::RobotState rs_init(*planning_init_rs_);
      robotState_mutex_.unlock();
      
      reset_robot_state(planning_init_rs_,req.ee_name,movePlan.trajectory_);
      robotState_mutex_.lock();
      ik_check_->reset_robot_state(*planning_init_rs_);
      moveit::core::RobotState rs_target(ik_check_->get_robot_state());
      robotState_mutex_.unlock();
      
      moveGroups_mutex_.lock();
      localMoveGroup = moveGroups_.at(req.ee_name);
      localMoveGroup->setJointValueTarget(rs_target);
      localMoveGroup->setStartState(rs_init);
      moveGroups_mutex_.unlock();
#endif
    }
    else
    {
      msg.data = "error";
    }
    
    hand_pub.at(local_capability).publish(msg); //publish on a topic when the trajectory is done
  
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
  
  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Executing plan for " << req.ee_name);

  moveit::planning_interface::MoveItErrorCode error_code;
  moveit::planning_interface::MoveGroup::Plan movePlan;
  
  movePlans_mutex_.lock();
  //NOTE: to be sure that no other execution is tried using this movePlan, use swap
  std::swap(movePlan,movePlans_.at(req.ee_name));
  movePlans_mutex_.unlock();
  
  // old execution method: does not allow for two trajectories at the same time
  moveGroups_mutex_.lock();
  if(!kinematics_only_)
    error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlan);
  moveGroups_mutex_.unlock();
  
  bool good_stop = waitForExecution(req.ee_name,movePlan.trajectory_);

  dual_manipulation_shared::ik_response msg;
  msg.seq=req.seq;
  msg.group_name = req.ee_name;
  
  if(good_stop)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }
  hand_pub.at(local_capability).publish(msg); //publish on a topic when the trajectory is done

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
	ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unknown capability \"" << capability_name << "\", returning");
	return false;
    }

    ik_control_capability_types capability;
    capability = capabilities_.type.at(capabilities_.from_name.at(capability_name));
    
    if(!busy.at(capability).count(ee_name))
    {
	ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unknown end effector \"" << ee_name << "\", returning");
	return false;
    }

    bool is_busy = false;

    // if I'm checking for a tree
    if(std::find(tree_names_list_.begin(),tree_names_list_.end(),ee_name) != tree_names_list_.end())
    {
      // if it's a capability which is not implemented yet for trees
      if(!capabilities_.implemented_for_trees.at(capability))
      {
	  ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Perform \"" << capability_name << "\" commands for each end-effector separately (tree version not implemented yet)! Returning");
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
      ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Already performing an ik_service of type " << capability_name << " for group " << ee_name);
    
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
	  bool check_collisions, use_clik, is_close;
	  if(req.command == capabilities_.name[ik_control_capabilities::PLAN])
	  {
	    check_collisions = true;
	    use_clik = false;
	    is_close = false;
	  }
	  else if(req.command == capabilities_.name[ik_control_capabilities::PLAN_NO_COLLISION])
	  {
	    check_collisions = false;
	    use_clik = false;
	    is_close = true;
	  }
	  else if(req.command == capabilities_.name[ik_control_capabilities::PLAN_BEST_EFFORT])
	  {
	    check_collisions = true;
	    use_clik = true;
	    is_close = false;
	  }
	  else if(req.command == capabilities_.name[ik_control_capabilities::PLAN_CLOSE_BEST_EFFORT])
	  {
	    check_collisions = true;
	    use_clik = true;
	    is_close = true;
	  }
	  else
	  {
	    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : this planning capability is not implemented yet!!!");
	    return false;
	  }
	  th = new std::thread(&ikControl::planning_thread,this, req, check_collisions, use_clik, is_close);
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
	  ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : this is strange - you shouldn't have come this far...!");
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

bool ikControl::moveHand(std::string& hand, std::vector< double >& q, std::vector< double >& t, trajectory_msgs::JointTrajectory& grasp_traj)
{
    // if an end-effector is not a hand
    if(!hand_actuated_joint_.count(hand))
        return true;
  // // do not fill the header if you're using different computers

  grasp_traj.joint_names.push_back(hand_actuated_joint_.at(hand));
  
  if (t.size() != q.size())
  {
    ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : timing vector size non compatible with joint vector size, using a default timing of 1 second");
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
  
  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : going back home...");
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
  
  // also open the hand(s) we're moving home, but don't wait for it(them)
  std::vector <double > q = {0.0};
  std::vector <double > t = {1.0/hand_max_velocity};
  for(auto& ee:chain_names)
  {
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : opening hand " << ee);
    trajectory_msgs::JointTrajectory grasp_traj;
    moveHand(ee,q,t,grasp_traj);
  }
  
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
  msg.group_name = req.ee_name;
  
  moveit::planning_interface::MoveGroup::Plan movePlan;

  moveit::planning_interface::MoveItErrorCode error_code(0);
  if(target_ok)
    error_code = moveGroups_.at(ee_name)->plan(movePlan);

  if(error_code.val != 1)
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to plan for \"" << group_name << "_home\", returning");
    msg.data = "error";
    hand_pub.at(local_capability).publish(msg);
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
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to forward \"" << group_name << "_home\" trajectory to the controller, returning");
    msg.data = "error";
    hand_pub.at(local_capability).publish(msg);
    map_mutex_.lock();
    busy.at(capabilities_.type.at(local_capability)).at(ee_name) = false;
    map_mutex_.unlock();
    // TODO: remove this!!
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now();
    end_time_mutex_.unlock();
    return;
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
  hand_pub.at(local_capability).publish(msg);
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
  
  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : \"" << req.attObject.object.id << "\" with \"" << req.ee_name << "\"");

  moveit::planning_interface::MoveItErrorCode error_code;

  dual_manipulation_shared::ik_response msg;
  msg.seq=req.seq;
  msg.group_name = req.ee_name;
  
  map_mutex_.lock();
  std::string grasped_obj;
  bool grasping = grasped_obj_map_.count(req.ee_name) != 0;
  if(grasping)
    grasped_obj = grasped_obj_map_.at(req.ee_name);
  map_mutex_.unlock();
  
  if(grasping)
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : end-effector " << req.ee_name << " already grasped an object (obj id: " << grasped_obj << "), returning");
    msg.data = "error";
    hand_pub.at(local_capability).publish(msg);
    // reset movement_end_time_ in order not to block planning
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now();
    end_time_mutex_.unlock();
    return;
  }
  
  // // get timed trajectory from waypoints
  moveit_msgs::RobotTrajectory trajectory;
  map_mutex_.lock();
  std::string group_name(group_map_.at(req.ee_name));
  map_mutex_.unlock();
  double allowed_distance = 2.5;
  std::vector<double> single_distances({0.5,0.5,0.5,1.0,2.0,2.0,2.0});
  ikCheck_mutex_.lock();
  double completed = computeTrajectoryFromWPs(trajectory,req.ee_pose,*ik_check_,group_name,req.ee_name,false,allowed_distance,single_distances);
  ikCheck_mutex_.unlock();
  if(completed != 1.0)
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to get trajectory from waypoints, returning");
    msg.data = "error";
    hand_pub.at(local_capability).publish(msg);
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
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to send trajectory to the controller, returning");
    msg.data = "error";
    hand_pub.at(local_capability).publish(msg);
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
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute approach trajectory, returning");
    msg.data = "error";
    hand_pub.at(local_capability).publish(msg);
    return;
  }
  
#ifdef SIMPLE_GRASP
  // // moveHand
  std::vector <double > q = {0.4,CLOSED_HAND};
  std::vector <double > t = {0.4/hand_max_velocity,0.5+1.0/hand_max_velocity};
  trajectory_msgs::JointTrajectory grasp_traj;
  moveHand(req.ee_name,q,t,grasp_traj);
  // // wait for hand moved
  good_stop = waitForHandMoved(req.ee_name,q.back(),grasp_traj);
#else
  // // wait for hand moved
  good_stop = waitForHandMoved(req.ee_name,req.grasp_trajectory.points.back().positions.at(0),req.grasp_trajectory);
#endif
  // I didn't make it
  if (!good_stop)
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute grasp trajectory, returning");
    msg.data = "error";
    hand_pub.at(local_capability).publish(msg);
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
  
  //ATTENTION: try to check for object in the scene: have they been set?
  bool object_attached = false;
  moveit_msgs::AttachedCollisionObject attObject_from_planning_scene;
  int attempts_left = 10;
  
  while(!object_attached && attempts_left-- > 0)
  {
    moveit_msgs::GetPlanningScene srv;
    uint32_t objects = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
    srv.request.components.components = objects;
    if(!scene_client_.call(srv))
      ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to call /get_planning_scene service - starting with an empty planning scene...");
    else
    {
      for(auto attObject:srv.response.scene.robot_state.attached_collision_objects)
	if(attObject.object.id == req.attObject.object.id)
	  if(std::find(allowed_collisions_.at(req.ee_name).begin(),allowed_collisions_.at(req.ee_name).end(),attObject.link_name) != allowed_collisions_.at(req.ee_name).end())
	  {
	    attObject_from_planning_scene = attObject;
	    object_attached = true;
	    break;
	  }
    }
    
    if(!object_attached)
    {
      ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : object \'" << req.attObject.object.id << "\' NOT FOUND in the planning scene (in the right place)!!! Sleeping 0.5s and checking again for " << attempts_left << " times...");
      usleep(500000);
    }
    else
      ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : object \'" << req.attObject.object.id << "\' FOUND in the planning scene (in the right place)!!!");

  }
  
  // we made it!
  msg.data = "done";
  hand_pub.at(local_capability).publish(msg);
  map_mutex_.lock();
  for(auto& obj:grasped_obj_map_)
    if(obj.second == req.attObject.object.id)
    {
      grasped_obj_map_.erase(obj.first);
      break;
    }
  grasped_obj_map_[req.ee_name] = req.attObject.object.id;
  objects_map_[req.attObject.object.id] = attObject_from_planning_scene;
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
  
  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : \"" << req.attObject.object.id << "\" from \"" << req.ee_name << "\"");

  moveit::planning_interface::MoveItErrorCode error_code;

  dual_manipulation_shared::ik_response msg;
  msg.seq=req.seq;
  msg.group_name = req.ee_name;
  //NOTE: never check collision for waypoints (at least for now)
  bool check_collisions = false;
  double allowed_distance = 25;
  std::vector<double> single_distances({1.0,3.0,3.0,3.0,5.0,5.0,5.0});
  
  // // get timed trajectory from waypoints
  moveit_msgs::RobotTrajectory trajectory;
  map_mutex_.lock();
  std::string group_name(group_map_.at(req.ee_name));
  map_mutex_.unlock();
  ikCheck_mutex_.lock();
  double completed = computeTrajectoryFromWPs(trajectory,req.ee_pose,*ik_check_,group_name,req.ee_name,check_collisions, allowed_distance, single_distances);
  ikCheck_mutex_.unlock();
  
#if !MOTION_PLAN_REQUEST_TESTING_ADVANCED
  // check if last waypoint is collision-free: if it's not, make sure to add one such WP, higher if needed!
  bool last_wp_collision_free = true;
  if(!trajectory.joint_trajectory.points.empty() && reset_robot_state(target_rs_,req.ee_name,trajectory))
  {
    bool self_collision_only = false;
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : checking robot for self-collisions in last found WP...");
    robotState_mutex_.lock();
    moveit::core::RobotState rs(*target_rs_);
    robotState_mutex_.unlock();
    ikCheck_mutex_.lock();
    last_wp_collision_free = ik_check_->is_state_collision_free(&rs,req.ee_name,self_collision_only);
    ikCheck_mutex_.unlock();
  }
  
  if(completed != 1.0 || !last_wp_collision_free)
  {
    if(completed != 1.0)
      ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to get trajectory from exact waypoints, trying again with approximate ones...");
    
    // if the only reason I entered is that last WP is in collision, I need to add a higher waypoint
    if(completed == 1.0 && !last_wp_collision_free)
    {
      ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : last WP was colliding, adding a new one, higher!");
      req.ee_pose.back().position.z += HIGH_UNGRASP_WP_IF_COLLIDING;
    }
    
    bool ik_ok = true;

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
      
      //TODO: change this to set_target with is_close flag true!!!
      ik_ok = set_close_target(req.ee_name,ee_poses,trials_nr,check_collisions,return_approximate_solution,allowed_distance);
      
      if(!ik_ok)
      {
	ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : set_close_target with an allowed joint-space distance of " << allowed_distance << "rads didn't work, trying again using CLIK and POSITION ONLY IK");
	bool use_clik = true;
	bool position_only_ik = true;
	bool is_close = true;
	ik_ok = set_target(req.ee_name,ee_poses,check_collisions,use_clik,position_only_ik,is_close);
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to get an IK solution for a close configuration, returning");
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
      ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to add the approximate waypoint to the trajectory, returning");
    }
    
    if(!ik_ok)
    {
      // revert everything and give an error
      reset_robot_state(target_rs_);
      msg.data = "error";
      hand_pub.at(local_capability).publish(msg);
      // reset movement_end_time_ in order not to block planning
      end_time_mutex_.lock();
      movement_end_time_ = ros::Time::now();
      end_time_mutex_.unlock();
      return;
    }
  }
#endif
  
  // // do not fill the header if you're using different computers
  
  bool good_stop = false;
  
#ifndef SIMPLE_GRASP
  // // align trajectories in time and check hand velocity limits
  computeHandTiming(trajectory,req);
  // // moveHand
  moveHand(req.ee_name,req.grasp_trajectory);
#elif SIMPLE_GRASP
  // // moveHand
  std::vector <double > q = {CLOSED_HAND, 0.0};
  std::vector <double > t = {0.0, 1.0/hand_max_velocity};
  trajectory_msgs::JointTrajectory grasp_traj;
  moveHand(req.ee_name,q,t,grasp_traj);
  // // wait for hand moved
  good_stop = waitForHandMoved(req.ee_name,q.back(),grasp_traj);
  // I didn't make it
  if (!good_stop)
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute ungrasp trajectory, returning");
    msg.data = "error";
    hand_pub.at(local_capability).publish(msg);
    // reset movement_end_time_ in order not to block planning
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now();
    end_time_mutex_.unlock();
    return;
  }
#endif

  map_mutex_.lock();
  std::string grasped_obj;
  bool grasping = grasped_obj_map_.count(req.ee_name) != 0;
  if(grasping)
    grasped_obj = grasped_obj_map_.at(req.ee_name);
  map_mutex_.unlock();
  
  if(grasping && (grasped_obj == req.attObject.object.id))
  {
    map_mutex_.lock();
    grasped_obj_map_.erase(req.ee_name);
    objects_map_.erase(grasped_obj);
    map_mutex_.unlock();
    
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
      ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : object with ID \"" << req.attObject.object.id << "\" is not grasped by \"" << req.ee_name << "\". Performing ungrasp action anyway");
    }
  }
  else
    ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : end-effector " << req.ee_name << " has grasped nothing or a different object than " << req.attObject.object.id << ", not detaching it in the planning scene");

if(completed > 0.0)
{
  // // execution of retreat
  moveit::planning_interface::MoveGroup::Plan movePlan;
  movePlan.trajectory_ = trajectory;
  moveGroups_mutex_.lock();
  error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlan);
  moveGroups_mutex_.unlock();
  if (error_code.val != 1)
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to send trajectory to the controller, returning");
    msg.data = "error";
    hand_pub.at(local_capability).publish(msg);
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
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute retreat trajectory, returning");
    msg.data = "error";
    hand_pub.at(local_capability).publish(msg);
    return;
  }
}
else
{
  end_time_mutex_.lock();
  movement_end_time_ = ros::Time::now();
  end_time_mutex_.unlock();
}
  
#ifndef SIMPLE_GRASP
  // // wait for hand moved
  good_stop = waitForHandMoved(req.ee_name,req.grasp_trajectory.points.back().positions.at(0),req.grasp_trajectory);
  // I didn't make it
  if (!good_stop)
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute ungrasp trajectory, returning");
    msg.data = "error";
    hand_pub.at(local_capability).publish(msg);
    return;
  }
#endif

  msg.data = "done";
  hand_pub.at(local_capability).publish(msg);
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

  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : resetting " << rs->getRobotModel()->getName());
  
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

  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : resetting " << rs->getRobotModel()->getName() << " with a trajectory for " << ee_name);

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

bool ikControl::set_target(std::string ee_name, std::vector< geometry_msgs::Pose > ee_poses, bool check_collisions, bool use_clik, bool position_only, bool is_close)
{
  std::unique_lock<std::mutex>(robotState_mutex_);
  
  // give an initial guess to ik_check_
  if(!ik_check_->reset_robot_state(*target_rs_))
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to reset ik_check");
    return false;
  }
  
  std::vector<std::vector<double>> solutions;
  const std::vector<double> initial_guess = std::vector<double>();
  bool return_approximate_solution = false;
  unsigned int attempts = 0;
  double timeout = 0.0;

  bool ik_ok;
  
  std::vector <ik_iteration_info> it_info;
  bool store_iterations = false;
  double allowed_distance = 100.0; // normally very high
  unsigned int trials_nr = 10;
  std::vector<double> single_distances;
  
  int max_counter = 4;
  // for close targets, reduce the allowed distance and increase the number of cyclic attempts (although it shouldn't be necessary...)!
  if(is_close)
  {
    //// this will also consider wrist dofs... better to just rely on single distances
    // allowed_distance = 5;
    max_counter = 10;
    map_mutex_.lock();
    single_distances = allowed_excursions_[ee_name];
    map_mutex_.unlock();
  }

  ik_ok = false;
  int counter = 0;
  while(!ik_ok && max_counter > 0)
  {
    max_counter--;
    
    ik_ok = ik_check_->find_closest_group_ik(ee_name,ee_poses,solutions,it_info,store_iterations,allowed_distance,single_distances,trials_nr,initial_guess,check_collisions,return_approximate_solution,attempts,timeout,use_clik,clik_threshold_);
    
    if(!ik_ok)
    {
      if(!ik_check_->reset_robot_state(*target_rs_))
      {
	ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to reset ik_check");
	return false;
      }
      
      if(!ik_ok && position_only)
      {
	ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : UNABLE to find a solution with find_group_ik and/or CLIK, trying again with position-only IK");
	if(!position_only_ik_check_->reset_robot_state(ik_check_->get_robot_state()))
	  ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to reset position_only_ik_check_");
	
	ik_ok = position_only_ik_check_->find_closest_group_ik(ee_name,ee_poses,solutions,it_info,store_iterations,allowed_distance,single_distances,trials_nr,initial_guess,check_collisions,return_approximate_solution,attempts,timeout,use_clik,clik_threshold_);
	
	if(!position_only_ik_check_->reset_robot_state(*target_rs_))
	{
	  ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to reset position_only_ik_check");
	  return false;
	}
	
	if(ik_ok && !ik_check_->reset_robot_state(position_only_ik_check_->get_robot_state()))
	{
	  ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to reset ik_check");
	  return false;
	}
      }
    }
    
//     if(!(check_collisions && !use_clik))
//     {
//       std::cout << "I'm trying... attempt #" << ++counter << " I am " << (ik_ok?"OK!!":"NOT ok!!!") << " | press n to return false..." << std::endl;
//       
//       char y; std::cin >> y;
//       if(y == 'n')
//       {
// 	moveit::core::RobotState rs(*planning_init_rs_);
// 	// ikCheck_mutex_.lock();
// 	ik_check_->reset_robot_state(rs);
// 	// ikCheck_mutex_.unlock();
// 	return false;
//       }
//     }
//     else
//       break;
  }
  
//   ik_ok = ik_check_->find_group_ik(ee_name,ee_poses,solutions,initial_guess,check_collisions,return_approximate_solution,attempts,timeout);
//   if(!ik_ok && use_clik)
//   {
//     ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : UNABLE to find a solution with find_group_ik, trying again with CLIK");
//     double clik_res = ik_check_->clik(ee_name,ee_poses,solutions,initial_guess,check_collisions,attempts,timeout);
//     ik_ok = clik_res > clik_threshold_;
//     if(!ik_ok)
//       ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : found CLIK solution up to " << 100.0*clik_res << "% of the initial gap, below the allowed threshold of " << clik_threshold_*100.0 << "%");
//   }
//   if(!ik_ok && position_only)
//   {
//     ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : UNABLE to find a solution with find_group_ik and/or CLIK, trying again with position-only IK");
//     if(!position_only_ik_check_->reset_robot_state(ik_check_->get_robot_state()))
//       ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to reset position_only_ik_check_");
//     
//     ik_ok = position_only_ik_check_->find_group_ik(ee_name,ee_poses,solutions,initial_guess,check_collisions,return_approximate_solution,attempts,timeout);
//     if(!ik_ok && use_clik)
//     {
//       ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : UNABLE to find a solution with position-only IK, trying again with CLIK");
//       double clik_res = position_only_ik_check_->clik(ee_name,ee_poses,solutions,initial_guess,check_collisions,attempts,timeout);
//       ik_ok = clik_res > clik_threshold_;
//       if(!ik_ok)
// 	ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : found POSITION ONLY CLIK solution up to " << 100.0*clik_res << "% of the initial gap, below the allowed threshold of " << clik_threshold_*100.0 << "%");
//     }
//     
//     if(ik_ok && !ik_check_->reset_robot_state(position_only_ik_check_->get_robot_state()))
//     {
//       ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to reset ik_check");
//       return false;
//     }
//   }
  
  if(!ik_ok)
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to find " << (position_only?"position_only ":"") << "IK for the requested pose " << (check_collisions?"":"NOT ") << "checking collisions and " << (use_clik?"":"NOT ") << "using CLIK");
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
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to reset ik_check");
    return false;
  }
  
  std::vector<std::vector<double>> solutions;
  std::vector <ik_iteration_info> it_info;
  bool store_iterations = false;
  std::vector<double> initial_guess;
  std::vector<double> single_distances;

  bool ik_ok = ik_check_->find_closest_group_ik(ee_name,ee_poses,solutions,it_info,store_iterations,allowed_distance,single_distances,trials_nr,initial_guess,check_collisions,return_approximate_solution);
  
  if(solutions.empty() || !ik_ok)
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to find IK for the requested pose");
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
	  ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unknown ik_target_type!!!");
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
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : requested set-target command \'" << req.command << "\' is not implemented!");
  
  busy.at(capabilities_.type.at(local_capability)).at(req.ee_name) = false;
}

bool ikControl::publishTrajectoryPath(const moveit_msgs::RobotTrajectory& trajectory_msg)
{
    std::string group_name;
    map_mutex_.lock();
    group_name = group_map_.at("full_robot");
    map_mutex_.unlock();
    robot_trajectory::RobotTrajectory trajectory(robot_model_,group_name);
    trajectory.setRobotTrajectoryMsg(*planning_init_rs_,trajectory_msg);
    ros::Duration dTs(0.1);
    ros::Duration time(0);
    
    static ros::Publisher joint_state_pub_;
    static bool pub_initialized(false);
    if (!pub_initialized)
    {
        joint_state_pub_ = node.advertise<sensor_msgs::JointState>("/joint_states",10);
        pub_initialized = true;
    }
    sensor_msgs::JointState js_msg;
    js_msg.name = trajectory_msg.joint_trajectory.joint_names;
    js_msg.header = trajectory_msg.joint_trajectory.header;
    ros::Duration total_time = trajectory_msg.joint_trajectory.points.back().time_from_start;
    
    ros::Rate rate(1.0/dTs.toSec());
    while(time < total_time)
    {
        time += dTs;
        if(time > total_time)
            time = total_time;
        trajectory.getStateAtDurationFromStart(time.toSec(),visual_rs_);
        
        js_msg.header.stamp = ros::Time::now();
        js_msg.position.clear();
        js_msg.velocity.clear();
        for(int i=0; i<js_msg.name.size(); i++)
        {
            js_msg.position.push_back(visual_rs_->getVariablePosition(js_msg.name.at(i)));
            js_msg.velocity.push_back(visual_rs_->getVariableVelocity(js_msg.name.at(i)));
        }
        joint_state_pub_.publish(js_msg);
        ros::spinOnce();
        
        rate.sleep();
    }
    
    return true;
}

std::string ikControl::findGroupName(const std::vector< std::string >& ee_list)
{
    std::string best_group;
    std::vector<std::string> ee_list_local;
    uint best_size = -1;
    map_mutex_.lock();
    
    for(auto& ee:ee_list)
    {
        if(std::find(tree_names_list_.begin(),tree_names_list_.end(),ee) != tree_names_list_.end())
            for(auto& c:tree_composition_.at(ee))
                ee_list_local.push_back(c);
        else
            ee_list_local.push_back(ee);
    }
    
    for(auto& t:tree_composition_)
    {
        bool found = true;
        for(auto& ee:ee_list_local)
        {
            found = (std::find(t.second.begin(),t.second.end(),ee) != t.second.end());
            if(!found)
                break;
        }
        if(!found)
            continue;
        
        if (t.second.size() < best_size)
        {
            best_size = t.second.size();
            best_group = t.first;
            if(best_size == ee_list_local.size())
                break;
        }
    }
    
    map_mutex_.unlock();
    
    return best_group;
}
