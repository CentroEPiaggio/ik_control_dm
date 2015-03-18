#include "ik_control.h"
#include "trajectory_utils.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <dual_manipulation_shared/parsing_utils.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#define EPS_VELOCITY 0.0007 // threshold on square sum : avg is 0.01 rad/s on each joint
#define EPS_POSITION 0.0007 // threshold on square sum : avg is 0.01 rad on each joint
#define MAX_HAND_VEL 2.0 // maximum hand velocity
#define HAND_EPS_POSITION 1.0/200.0
#define SIMPLE_GRASP 1

using namespace dual_manipulation::ik_control;

ikControl::ikControl()
{
    busy["left_hand"]=false;
    busy["right_hand"]=false;
    busy["both_hands"]=false;

    hand_pub["exec"]["left_hand"] = node.advertise<std_msgs::String>("/ik_control/left_hand/action_done",1,this);
    hand_pub["exec"]["right_hand"] = node.advertise<std_msgs::String>("/ik_control/right_hand/action_done",1,this);
    hand_pub["exec"]["both_hands"] = node.advertise<std_msgs::String>("/ik_control/both_hands/action_done",1,this);
    hand_pub["plan"]["left_hand"] = node.advertise<std_msgs::String>("/ik_control/left_hand/planning_done",1,this);
    hand_pub["plan"]["right_hand"] = node.advertise<std_msgs::String>("/ik_control/right_hand/planning_done",1,this);
    hand_pub["plan"]["both_hands"] = node.advertise<std_msgs::String>("/ik_control/both_hands/planning_done",1,this);
    hand_pub["check"]["left_hand"] = node.advertise<std_msgs::String>("/ik_control/left_hand/check_done",1,this);
    hand_pub["check"]["right_hand"] = node.advertise<std_msgs::String>("/ik_control/right_hand/check_done",1,this);
    hand_pub["check"]["both_hands"] = node.advertise<std_msgs::String>("/ik_control/both_hands/check_done",1,this);
    hand_pub["grasp"]["left_hand"] = node.advertise<std_msgs::String>("/ik_control/left_hand/grasp_done",1,this);
    hand_pub["grasp"]["right_hand"] = node.advertise<std_msgs::String>("/ik_control/right_hand/grasp_done",1,this);
    
    traj_pub_["left_hand"] = node.advertise<trajectory_msgs::JointTrajectory>("/left_arm/joint_trajectory_controller/command",1,this);
    traj_pub_["right_hand"] = node.advertise<trajectory_msgs::JointTrajectory>("/right_arm/joint_trajectory_controller/command",1,this);
    
    hand_synergy_pub_["left_hand"] = node.advertise<trajectory_msgs::JointTrajectory>("/left_hand/joint_trajectory_controller/command",1,this);
    hand_synergy_pub_["right_hand"] = node.advertise<trajectory_msgs::JointTrajectory>("/right_hand/joint_trajectory_controller/command",1,this);
    
    kinematics_plugin_["left_hand"] = new kdl_kinematics_plugin::KDLKinematicsPlugin();
    kinematics_plugin_["right_hand"] = new kdl_kinematics_plugin::KDLKinematicsPlugin();
    
    group_map_["left_hand"] = "left_hand_arm";
    group_map_["right_hand"] = "right_hand_arm";
    group_map_["both_hands"] = "dual_hand_arm";

    controller_map_["left_hand"] = "/left_arm/joint_trajectory_controller/follow_joint_trajectory/";
    controller_map_["right_hand"] = "/right_arm/joint_trajectory_controller/follow_joint_trajectory/";
    
    hand_actuated_joint_["left_hand"] = "left_hand_synergy_joint";
    hand_actuated_joint_["right_hand"] = "right_hand_synergy_joint";
    
    moveGroups_["left_hand"] = new move_group_interface::MoveGroup(group_map_.at("left_hand"));
    moveGroups_["right_hand"] = new move_group_interface::MoveGroup(group_map_.at("right_hand"));
    moveGroups_["both_hands"] = new move_group_interface::MoveGroup(group_map_.at("both_hands"));
    
    ee_map_["left_hand"] = moveGroups_.at("left_hand")->getEndEffectorLink();
    ee_map_["right_hand"] = moveGroups_.at("right_hand")->getEndEffectorLink();
    
    movePlans_["left_hand"];
    movePlans_["right_hand"];
    movePlans_["both_hands"];
    
    // NOTE: attempted value of search_discretization: it's not clear what it is used for
    kinematics_plugin_.at("left_hand")->initialize("robot_description",group_map_.at("left_hand"),"world",ee_map_.at("left_hand"),0.005);
    kinematics_plugin_.at("right_hand")->initialize("robot_description",group_map_.at("right_hand"),"world",ee_map_.at("right_hand"),0.005);
    
    ik_serviceClient_ = node.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    
    isInitialized_ = true;
    
    // give me all robot links in order to set allowed collisions map
    std::vector<std::string> links = moveGroups_.at("left_hand")->getCurrentState()->getRobotModel()->getLinkModelNamesWithCollisionGeometry();
    
    //TODO: get these (as an array of strings to check for as initial part of the joint names) from parameter server
    std::string left_hand="left_hand";
    std::string right_hand="right_hand";

    allowed_collisions_[left_hand].push_back("left_arm_7_link");
    allowed_collisions_[right_hand].push_back("right_arm_7_link");
    for (auto item:links)
      if (item.compare(0,left_hand.size(),left_hand.c_str()) == 0)
	allowed_collisions_[left_hand].push_back(item);
      else if (item.compare(0,right_hand.size(),right_hand.c_str()) == 0)
	allowed_collisions_[right_hand].push_back(item);

    position_threshold=EPS_POSITION;
    velocity_threshold=EPS_VELOCITY;
    hand_max_velocity=MAX_HAND_VEL;
    hand_position_threshold=HAND_EPS_POSITION;
    
    if (node.getParam("ik_control_parameters", ik_control_params))
        parseParameters( ik_control_params);
}

void ikControl::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    parseSingleParameter(params,position_threshold,"position_threshold");
    parseSingleParameter(params,velocity_threshold,"velocity_threshold");
    parseSingleParameter(params,hand_max_velocity,"hand_max_velocity");
    parseSingleParameter(params,hand_position_threshold,"hand_position_threshold");
    parseSingleParameter(params,kinematics_only_,"kinematics_only");

    std::vector<std::string> names_list({"fake_name_1","fake_name_2"});
    parseSingleParameter(params,names_list,"group_names");
    
    parseSingleParameter(params,group_map_,"group_map",names_list);
    
    // planner parameters
    std::string planner_id = "RRTstarkConfigDefault";
    double planning_time = 1.0;
    double goal_position_tolerance = 0.005;
    double goal_orientation_tolerance = 0.005;
    double goal_joint_tolerance = 0.005;
    std::vector<double> workspace_bounds({-1.2,-1.5,0.1,0.2,1.5,1.5});
    if(params.hasMember("motion_planner"))
    {
      parseSingleParameter(params["motion_planner"],planner_id,"planner_id");
      parseSingleParameter(params["motion_planner"],planning_time,"planning_time");
      parseSingleParameter(params["motion_planner"],goal_position_tolerance,"goal_position_tolerance");
      parseSingleParameter(params["motion_planner"],goal_orientation_tolerance,"goal_orientation_tolerance");
      parseSingleParameter(params["motion_planner"],goal_joint_tolerance,"goal_joint_tolerance");
      parseSingleParameter(params["motion_planner"],workspace_bounds,"workspace_bounds",6);
    }
    for(auto item:moveGroups_)
    {
      item.second->setPlannerId(planner_id);
      item.second->setPlanningTime(planning_time);
      item.second->setGoalPositionTolerance(goal_position_tolerance);
      item.second->setGoalOrientationTolerance(goal_orientation_tolerance);
      item.second->setGoalJointTolerance(goal_joint_tolerance);
      item.second->setWorkspace(-1.2,-1.5,0.1,0.2,1.5,1.5);
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
  double vel,dist;
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

  while(counter<200)
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

bool ikControl::waitForExecution(std::string ee_name)
{
  ROS_INFO_STREAM("ikControl::waitForExecution : entered");
  
  if (movePlans_.at(ee_name).trajectory_.joint_trajectory.points.size() == 0)
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

  control_msgs::FollowJointTrajectoryActionResultConstPtr pt;
  ros::Duration timeout = movePlans_.at(ee_name).trajectory_.joint_trajectory.points.back().time_from_start;
  if(controller_map_.count(ee_name) != 0)
  {
    // only do this if a controller exists - use a scaled timeout
    timeout = timeout*3.0;
    pt = ros::topic::waitForMessage<control_msgs::FollowJointTrajectoryActionResult>(controller_map_.at(ee_name) + "result",node,timeout);
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

  std::vector<std::string> joints = moveGroups_.at(ee_name)->getActiveJoints();
  std::vector<double> q,Dq,goal_q;
  q.reserve(joints.size());
  Dq.reserve(joints.size());
  
  // (remember that the goal position is the last pose of the movePlan)
  // TODO: when a plan has not been computed correctly, this could give a std::out_of_range exception: fix this
  trajectory_msgs::JointTrajectoryPoint point = movePlans_.at(ee_name).trajectory_.joint_trajectory.points.back();
  for(auto joint:joints)
    for(int i=0; i<movePlans_.at(ee_name).trajectory_.joint_trajectory.joint_names.size(); i++)
      if (joint == movePlans_.at(ee_name).trajectory_.joint_trajectory.joint_names.at(i))
	goal_q.push_back(point.positions.at(i));
  
  // goal size and joints size MUST be equal: check
  assert(goal_q.size() == joints.size());

  int counter = 0;
  double vel,dist;
  bool good_stop = false;
  
  sensor_msgs::JointStateConstPtr joint_states;
  
  while(counter<200)
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
      // q.push_back(*(moveGroups_.at(ee_name)->getCurrentState()->getJointPositions(joint)));
      // Dq.push_back(*(moveGroups_.at(ee_name)->getCurrentState()->getJointVelocities(joint)));
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
    counter++;
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

  moveit_msgs::GetPositionIK::Request service_request;
  moveit_msgs::GetPositionIK::Response service_response; 
  service_request.ik_request.group_name = group_map_.at(req.ee_name);
  service_request.ik_request.pose_stamped.header.frame_id = "world";  
  service_request.ik_request.pose_stamped.pose = req.ee_pose.at(0);
  service_request.ik_request.ik_link_name = ee_map_.at(req.ee_name);
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
  
  if(service_response.error_code.val == 1)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }
  hand_pub.at("check").at(req.ee_name).publish(msg); //publish on a topic when the IK check is done

  busy.at(req.ee_name)=false;
  
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
	  hand_pub.at("plan").at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done
	  busy.at(req.ee_name)=false;
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
	  hand_pub.at("plan").at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done
	  busy.at(req.ee_name)=false;
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
      hand_pub.at("plan").at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done

      busy.at(req.ee_name)=false;

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
    
    hand_pub.at("plan").at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done
  
    busy.at(req.ee_name)=false;
    
    return;
}

void ikControl::execute_plan(dual_manipulation_shared::ik_service::Request req)
{
  ROS_INFO("IKControl::execute_plan: Executing plan for %s",req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;
  
  // old execution method: does not allow for two trajectories at the same time
  error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlans_.at(req.ee_name));
  
  bool good_stop = waitForExecution(req.ee_name);
  
  if(good_stop)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }
  hand_pub.at("exec").at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done

  busy.at(req.ee_name)=false;
  
  return;
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
    
    if (!isInitialized_)
    {
      ROS_WARN("IKControl::perform_ik: robot model is not initialized - initialize it first!");
      return false;
    }

    if(!busy.count(req.ee_name))
    {
	ROS_ERROR("IKControl::perform_ik: Unknown end effector %s, returning",req.ee_name.c_str());
	return false;
    }

    if(req.ee_name == "both_hands" && ((req.command == "ik_check") || (req.command == "grasp") || (req.command == "ungrasp")))
    {
	ROS_ERROR("IKControl::perform_ik: Perform %s commands for each hand separately! Returning",req.command.c_str());
	return false;
    }

    // check for correctness when using one or both hands
    bool plan_bool;
    if (req.ee_name == "both_hands") 
      plan_bool = !(busy.at("left_hand") || busy.at("right_hand"));
    else
      plan_bool = !(busy.at(req.ee_name));
    
    if(plan_bool && !busy.at("both_hands"))
    {
	std::thread* th;
	busy.at(req.ee_name)=true;
	if(req.command == "plan")
	{
	  th = new std::thread(&ikControl::planning_thread,this, req);
	}
	else if(req.command == "ik_check")
	{
	  th = new std::thread(&ikControl::ik_check_thread,this, req);
	}
	else if(req.command == "execute")
	{
	  th = new std::thread(&ikControl::execute_plan,this, req);
	}
	else if(req.command == "home")
	{
	  th = new std::thread(&ikControl::simple_homing,this, req.ee_name);
	}
	else if(req.command == "grasp")
	{
	  th = new std::thread(&ikControl::grasp,this, req);
	}
	else if(req.command == "ungrasp")
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
    delete kinematics_plugin_.at("left_hand");
    delete kinematics_plugin_.at("right_hand");
    
    delete moveGroups_.at("left_hand");
    delete moveGroups_.at("right_hand");
    delete moveGroups_.at("both_hands");
    
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

  moveGroups_.at(ee_name)->setNamedTarget( group_map_.at(ee_name) + "_home" );
  moveGroups_.at(ee_name)->setStartStateToCurrentState();
  
  moveit::planning_interface::MoveItErrorCode error_code = moveGroups_.at(ee_name)->plan(movePlans_.at(ee_name));
  if(error_code.val != 1)
  {
    ROS_ERROR_STREAM("ikControl::simple_homing : unable to plan for \"" << group_map_.at(ee_name) << "_home\", returning");
    msg.data = "error";
    hand_pub.at("exec").at(ee_name).publish(msg);
    return;
  }
  
  error_code = moveGroups_.at(ee_name)->asyncExecute(movePlans_.at(ee_name));
  if(error_code.val != 1)
  {
    ROS_ERROR_STREAM("ikControl::simple_homing : unable to forward \"" << group_map_.at(ee_name) << "_home\" trajectory to the controller, returning");
    msg.data = "error";
    hand_pub.at("exec").at(ee_name).publish(msg);
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
  
  bool good_stop = waitForExecution(ee_name);
  if(!good_stop)
  {
    msg.data = "error";
    hand_pub.at("exec").at(ee_name).publish(msg);
    return;
  }

  msg.data = "done";
  hand_pub.at("exec").at(ee_name).publish(msg);
  busy.at(ee_name) = false;
  
  return;
}

void ikControl::grasp(dual_manipulation_shared::ik_service::Request req)
{
// // //   TODO: check if this is actually necessary as we're not using planning, but it may check for collisions on its own
// // //   removeObject(req.attObject.object.id);
  
  ROS_INFO("IKControl::grasp: %s with %s",req.attObject.object.id.c_str(),req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;
  
  // // get timed trajectory from waypoints
  moveit_msgs::RobotTrajectory trajectory;
  if(!computeTrajectoryFromWPs(trajectory,req.ee_pose,moveGroups_.at(req.ee_name)))
  {
    ROS_ERROR("ikControl::grasp : unable to get trajectory from waypoints, returning");
    msg.data = "error";
    hand_pub.at("grasp").at(req.ee_name).publish(msg);
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
    hand_pub.at("grasp").at(req.ee_name).publish(msg);
    return;
  }
  
#ifndef SIMPLE_GRASP
  moveHand(req.ee_name,req.grasp_trajectory);
#endif
  
  // // wait for approach
  bool good_stop = waitForExecution(req.ee_name);
  // I didn't make it
  if (!good_stop)
  {
    ROS_ERROR("ikControl::grasp : unable to execute approach trajectory, returning");
    msg.data = "error";
    hand_pub.at("grasp").at(req.ee_name).publish(msg);
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
    hand_pub.at("grasp").at(req.ee_name).publish(msg);
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
  hand_pub.at("grasp").at(req.ee_name).publish(msg);
  busy.at(req.ee_name) = false;
  
  return;
}

void ikControl::ungrasp(dual_manipulation_shared::ik_service::Request req)
{
  ROS_INFO("IKControl::ungrasp: %s from %s",req.attObject.object.id.c_str(),req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;
  
  // // get timed trajectory from waypoints
  moveit_msgs::RobotTrajectory trajectory;
  if(!computeTrajectoryFromWPs(trajectory,req.ee_pose,moveGroups_.at(req.ee_name)))
  {
    ROS_ERROR("ikControl::grasp : unable to get trajectory from waypoints, returning");
    msg.data = "error";
    hand_pub.at("grasp").at(req.ee_name).publish(msg);
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
    hand_pub.at("grasp").at(req.ee_name).publish(msg);
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
    hand_pub.at("grasp").at(req.ee_name).publish(msg);
    return;
  }
  
  // // wait for retreat
  good_stop = waitForExecution(req.ee_name);
  // I didn't make it
  if (!good_stop)
  {
    ROS_ERROR("ikControl::ungrasp : unable to execute retreat trajectory, returning");
    msg.data = "error";
    hand_pub.at("grasp").at(req.ee_name).publish(msg);
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
    hand_pub.at("grasp").at(req.ee_name).publish(msg);
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
  hand_pub.at("grasp").at(req.ee_name).publish(msg);
  busy.at(req.ee_name) = false;
  
  return;
}

