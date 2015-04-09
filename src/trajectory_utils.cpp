#include "trajectory_utils.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

bool splitFullRobotPlan(std::map<std::string,move_group_interface::MoveGroup*> moveGroups_, std::map<std::string,moveit::planning_interface::MoveGroup::Plan> movePlans_)
{
  int start_left,end_left,start_right,end_right;
  
  std::vector<std::string> active_joints_both = moveGroups_.at("both_hands")->getActiveJoints();
  std::vector<std::string> active_joints_left = moveGroups_.at("left_hand")->getActiveJoints();
  std::vector<std::string> active_joints_right = moveGroups_.at("right_hand")->getActiveJoints();
  
  std::vector <std::string >::iterator tmp;
  
  tmp = std::find(active_joints_both.begin(), active_joints_both.end(), active_joints_left.front());
  if (tmp != active_joints_both.end())
    start_left = tmp - active_joints_both.begin();
  else
  {
    ROS_ERROR("Left joint %s not found in full_robot group",active_joints_left.front().c_str());
    return false;
  }
  
  tmp = std::find(active_joints_both.begin(), active_joints_both.end(), active_joints_left.back());
  if (tmp != active_joints_both.end())
    end_left = tmp - active_joints_both.begin();
  else
  {
    ROS_ERROR("Left joint %s not found in full_robot group",active_joints_left.back().c_str());
    return false;
  }
  
  tmp = std::find(active_joints_both.begin(), active_joints_both.end(), active_joints_right.front());
  if (tmp != active_joints_both.end())
    start_right = tmp - active_joints_both.begin();
  else
  {
    ROS_ERROR("Right joint %s not found in full_robot group",active_joints_left.front().c_str());
    return false;
  }
  
  tmp = std::find(active_joints_both.begin(), active_joints_both.end(), active_joints_right.back());
  if (tmp != active_joints_both.end())
    end_right = tmp - active_joints_both.begin();
  else
  {
    ROS_ERROR("Right joint %s not found in full_robot group",active_joints_left.back().c_str());
    return false;
  }
  
  trajectory_msgs::JointTrajectoryPoint tmp_traj;
  
  // std::cout << "start_left | end_left | start_right | end_right = " << start_left << " | " << end_left << " | " << start_right << " | " << end_right << std::endl;
  
  for (auto item:movePlans_.at("both_hands").trajectory_.joint_trajectory.points)
  {
    // clear trajectory point
    tmp_traj.positions.clear();
    tmp_traj.velocities.clear();
    tmp_traj.accelerations.clear();
    tmp_traj.effort.clear();
    tmp_traj.time_from_start = item.time_from_start;
    
    // get left joint values
    if (!item.positions.empty())
      tmp_traj.positions.insert(tmp_traj.positions.end(),item.positions.begin()+start_left,item.positions.begin()+end_left+1);
    if (!item.velocities.empty())
      tmp_traj.velocities.insert(tmp_traj.velocities.end(),item.velocities.begin()+start_left,item.velocities.begin()+end_left+1);
    if (!item.accelerations.empty())
      tmp_traj.accelerations.insert(tmp_traj.accelerations.end(),item.accelerations.begin()+start_left,item.accelerations.begin()+end_left+1);
    if (!item.effort.empty())
      tmp_traj.effort.insert(tmp_traj.effort.end(),item.effort.begin()+start_left,item.effort.begin()+end_left+1);
    
    // std::cout << "left_hand trajectory point" << std::endl;
    // std::cout << tmp_traj << std::endl;
    
    // push them in the left hand trajectory
    movePlans_.at("left_hand").trajectory_.joint_trajectory.points.push_back(tmp_traj);
    
    // clear trajectory point
    tmp_traj.positions.clear();
    tmp_traj.velocities.clear();
    tmp_traj.accelerations.clear();
    tmp_traj.effort.clear();
    tmp_traj.time_from_start = item.time_from_start;
    
    // get right joint values
    if (!item.positions.empty())
      tmp_traj.positions.insert(tmp_traj.positions.end(),item.positions.begin()+start_right,item.positions.begin()+end_right+1);
    if (!item.velocities.empty())
      tmp_traj.velocities.insert(tmp_traj.velocities.end(),item.velocities.begin()+start_right,item.velocities.begin()+end_right+1);
    if (!item.accelerations.empty())
      tmp_traj.accelerations.insert(tmp_traj.accelerations.end(),item.accelerations.begin()+start_right,item.accelerations.begin()+end_right+1);
    if (!item.effort.empty())
      tmp_traj.effort.insert(tmp_traj.effort.end(),item.effort.begin()+start_right,item.effort.begin()+end_right+1);
    
    // std::cout << "right_hand trajectory point" << std::endl;
    // std::cout << tmp_traj << std::endl;
    
    // push them in the right hand trajectory
    movePlans_.at("right_hand").trajectory_.joint_trajectory.points.push_back(tmp_traj);    
  }
  
  movePlans_.at("left_hand").trajectory_.joint_trajectory.joint_names.clear();
  movePlans_.at("left_hand").trajectory_.joint_trajectory.joint_names.insert( movePlans_.at("left_hand").trajectory_.joint_trajectory.joint_names.end(), active_joints_left.begin(), active_joints_left.end() );
  movePlans_.at("right_hand").trajectory_.joint_trajectory.joint_names.clear();
  movePlans_.at("right_hand").trajectory_.joint_trajectory.joint_names.insert( movePlans_.at("right_hand").trajectory_.joint_trajectory.joint_names.end(), active_joints_right.begin(), active_joints_right.end() );
  
  return true;
}

double computeTrajectoryFromWPs(moveit_msgs::RobotTrajectory& trajectory, const std::vector< geometry_msgs::Pose >& waypoints, moveit::planning_interface::MoveGroup* moveGroup, bool avoid_collisions, double eef_step, double jump_threshold)
{
  // compute waypoints
  moveit_msgs::MoveItErrorCodes error_code;
  double completed = moveGroup->computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory,avoid_collisions,&error_code);

  if(completed < 1.0 || error_code.val != 1)
  {
    ROS_ERROR_STREAM("trajectory_utils::computeTrajectoryFromWPs : error in computing trajectory | error_code.val=" << error_code.val << " | completed part = " << completed*100.0 << "%");
    return completed;
  }
  
  // interpolate them
  //NOTE: robot_traj, built on robot_model, contains the full robot; trajectory, instead, is only for the group joints
  robot_trajectory::RobotTrajectory robot_traj(moveGroup->getCurrentState()->getRobotModel(),moveGroup->getName());
  robot_traj.setRobotTrajectoryMsg(*(moveGroup->getCurrentState()),trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  // compute time stamps
  if (!iptp.computeTimeStamps(robot_traj))
  {
    ROS_ERROR("trajectory_utils::computeTrajectoryFromWPs : unable to compute time stamps for the trajectory");
    return -1;
  }
  robot_traj.getRobotTrajectoryMsg(trajectory);

  return completed;
}

bool computeHandTiming(moveit_msgs::RobotTrajectory& trajectory, dual_manipulation_shared::ik_service::Request& req)
{
  if (trajectory.joint_trajectory.points.size() == req.grasp_trajectory.points.size())
  {
    for (int i=0; i<trajectory.joint_trajectory.points.size(); ++i)
    {
      req.grasp_trajectory.points.at(i).time_from_start = trajectory.joint_trajectory.points.at(i).time_from_start;
    }
  }
  else
  {
    ROS_WARN_STREAM("trajectory_utils::computeHandTiming : unequal length arm (" << trajectory.joint_trajectory.points.size() << ") and hand (" << req.grasp_trajectory.points.size() << ") trajectories: interpolating hand trajectory");
    ros::Duration delta_t = trajectory.joint_trajectory.points.back().time_from_start*(1.0/req.grasp_trajectory.points.size());
    for (int i=0; i<req.grasp_trajectory.points.size(); ++i)
    {
      req.grasp_trajectory.points.at(i).time_from_start = delta_t*(i+1);
    }
  }
  
  //TODO: impose velocity limits
}


