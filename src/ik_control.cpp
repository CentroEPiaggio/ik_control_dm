#include "ik_control.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <thread>

using namespace dual_manipulation::ik_control;

ikControl::ikControl()
{
    busy["left_hand"]=false;
    busy["right_hand"]=false;

    hand_pub["left_hand"] = node.advertise<std_msgs::String>("/ik_control/left_hand/action_done",0,this);
    hand_pub["right_hand"] = node.advertise<std_msgs::String>("/ik_control/right_hand/action_done",0,this);
    
    robot_state_publisher_ = node.advertise<moveit_msgs::DisplayRobotState>( "/ik_control/robot_state", 1 );
    
    kinematics_plugin_["left_hand"] = new kdl_kinematics_plugin::KDLKinematicsPlugin();
    kinematics_plugin_["right_hand"] = new kdl_kinematics_plugin::KDLKinematicsPlugin();
    
    group_map_["left_hand"] = "left_hand_arm";
    group_map_["right_hand"] = "right_hand_arm";
    group_map_["both_hands"] = "full_robot";
    
    moveGroups_["left_hand"] = new move_group_interface::MoveGroup(group_map_.at("left_hand"));
    moveGroups_["right_hand"] = new move_group_interface::MoveGroup(group_map_.at("right_hand"));
    moveGroups_["both_hands"] = new move_group_interface::MoveGroup(group_map_.at("both_hands"));
    
    movePlans_["left_hand"];
    movePlans_["right_hand"];
    movePlans_["both_hands"];
    
    // NOTE: attempted value of search_discretization: it's not clear what it is used for
    kinematics_plugin_.at("left_hand")->initialize("robot_description","left_hand_arm","world","left_hand_palm_link",0.005);
    kinematics_plugin_.at("right_hand")->initialize("robot_description","right_hand_arm","world","right_hand_palm_link",0.005);
    
    isInitialized_ = true;
}

void ikControl::ik_check_thread(dual_manipulation_shared::ik_service::Request req)
{
  ROS_INFO("IKControl::ik_check_thread: Thread spawned! Computing IK for %s",req.ee_name.c_str());
  
  kdl_kinematics_plugin::KDLKinematicsPlugin* kin_ptr = kinematics_plugin_[req.ee_name.c_str()];
  
  std::vector <double> ik_seed_state;
  double timeout = 10.0;
  std::vector <double> solution;
  moveit_msgs::MoveItErrorCodes error_code;
  
  ik_seed_state.resize(kin_ptr->getJointNames().size());
  for (auto item:ik_seed_state)
    item = 0.0;
  
  kin_ptr->searchPositionIK(req.ee_pose, ik_seed_state, timeout, solution, error_code);
  
  ROS_INFO_STREAM("IKControl::ik_check_thread: error_code.val = " << error_code.val << std::endl);
  for (auto item:solution)
    std::cout << item << " | ";
  std::cout << std::endl;
  
  if(error_code.val == 1)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }
  hand_pub[req.ee_name].publish(msg); //publish on a topic when the IK check is done

  busy[req.ee_name]=false;
  
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

    if ( localMoveGroup->setPoseTarget(req.ee_pose) )
//     if ( localMoveGroup->setPoseTarget( localMoveGroup->getCurrentPose().pose ) )
    {
      ROS_INFO_STREAM("IKControl::planning_thread: Target set correctly!" << std::endl);
    }
    else
    {
      ROS_WARN_STREAM("IKControl::planning_thread: Unable to set target pose\n");
      
    }
    // unconmment to set a different tolerance (to 0.005 m / 0.005 rad = 0.5 degree in this case)
    localMoveGroup->setGoalTolerance(0.005);
    
    moveit::planning_interface::MoveGroup::Plan* movePlan = &(movePlans_.at(req.ee_name));
    localMoveGroup->plan(*movePlan);
    
    ROS_INFO_STREAM("movePlan traj size: " << movePlan->trajectory_.joint_trajectory.points.size() << std::endl);
    for (int i=0; i<movePlan->trajectory_.joint_trajectory.points.size(); ++i)
    {
      ROS_DEBUG_STREAM(movePlan->trajectory_.joint_trajectory.points.at(i) << std::endl);
      // std::cout << movePlan->trajectory_.joint_trajectory.points.at(i) << std::endl;
    }
    
    ROS_DEBUG_STREAM("pos [x y z]: " << req.ee_pose.position.x << " " << req.ee_pose.position.y << " " << req.ee_pose.position.z << std::endl);
    ROS_DEBUG_STREAM("orient [x y z w]: "  << req.ee_pose.orientation.x << " " << req.ee_pose.orientation.y << " " << req.ee_pose.orientation.z << " " << req.ee_pose.orientation.w << std::endl);

    // /* get a robot state message describing the pose in robot_state_ */
    // moveit_msgs::DisplayRobotState robotStateMsg;
    // robot_state::robotStateToRobotStateMsg(*robot_state_, robotStateMsg.state);
    // /* send the message to the RobotState display */
    // robot_state_publisher_.publish( robotStateMsg );

    /* let ROS send the message */
    ros::spinOnce();

    msg.data = "done";
    hand_pub.at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done
  
    busy.at(req.ee_name)=false;
    
    return;
}

void ikControl::execute_plan(dual_manipulation_shared::ik_service::Request req)
{
  ROS_INFO("IKControl::execute_plan: Executing plan for %s",req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;
  error_code = moveGroups_.at(req.ee_name)->execute(movePlans_.at(req.ee_name));

  if(error_code.val == 1)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }
  hand_pub.at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done

  busy.at(req.ee_name)=false;
  
  return;
}

bool ikControl::perform_ik(dual_manipulation_shared::ik_service::Request& req)
{
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
    if(!busy[req.ee_name])
    {
	busy[req.ee_name]=true;
	if(req.command == "plan")
	{
	  std::thread* th = new std::thread(&ikControl::planning_thread,this, req);
	}
	else if(req.command == "ik_check")
	{
	  std::thread* th = new std::thread(&ikControl::ik_check_thread,this, req);
	}
	else if(req.command == "execute")
	{
	  execute_plan(req);
	}
	else
	{
	  ROS_WARN("IKControl::perform_ik: Unknown command: %s",req.command.c_str());
	  return false;
	}
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
    delete kinematics_plugin_["left_hand"];
    delete kinematics_plugin_["right_hand"];
    
    delete moveGroups_["left_hand"];
    delete moveGroups_["right_hand"];
    delete moveGroups_["both_hands"];
}
