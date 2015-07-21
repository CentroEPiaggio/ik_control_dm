#include "trajectory_utils.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "ik_check_capability/ik_check_capability.h"

#define INTERPOLATING_WPs_NR 0
#define CLASS_NAMESPACE "trajectory_utils::"

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

double computeTrajectoryFromWPs(moveit_msgs::RobotTrajectory& trajectory, const std::vector< geometry_msgs::Pose >& waypoints, dual_manipulation::ik_control::ikCheckCapability& ikCheck, std::string group_name, std::string ee_name, bool avoid_collisions, double allowed_distance, std::vector<double>& single_distances)
{
  // compute waypoints
  double completed = 0.0;
  int i;
  std::vector<std::vector<double>> solutions;
  std::vector<double> initial_guess;
  
  // store starting robot state and add it to the trajectory
  moveit::core::RobotStatePtr init_rs(new moveit::core::RobotState(ikCheck.get_robot_state()));
  add_wp_to_traj(init_rs,group_name,trajectory);
  
  std::vector <dual_manipulation::ik_control::ik_iteration_info > it_info;
  bool store_iterations = false;
  // TODO: make this general
  unsigned int trials_nr = 1;
  bool return_approximate_solution = false;
  unsigned int attempts = 0;
  double timeout = 0.0;
  bool use_clik = false;
  double clik_percentage = 0.1;
  const std::map <std::string, std::string > allowed_collisions = std::map< std::string, std::string >();
  
  std::vector<moveit::core::RobotStatePtr> rs_vec;
  rs_vec.push_back(init_rs);
  
  for(i=0; i<waypoints.size(); i++)
  {
    std::vector<geometry_msgs::Pose> ee_poses({waypoints.at(i)});
    // if(!ikCheck.find_group_ik(ee_name,ee_poses,solutions,initial_guess,avoid_collisions))
    if(!ikCheck.find_closest_group_ik(ee_name,ee_poses,solutions,it_info,store_iterations,allowed_distance,single_distances,trials_nr,initial_guess,avoid_collisions,return_approximate_solution,attempts,timeout,use_clik,clik_percentage,allowed_collisions))
      break;
    
    moveit::core::RobotStatePtr rs(new moveit::core::RobotState(ikCheck.get_robot_state()));
    rs_vec.push_back(rs);
    
    if(!add_wp_to_traj(rs,group_name,trajectory))
      break;
  }
  completed = (double)i/waypoints.size();

  // show an error if I didn't complete the trajectory, but continue with the parametrization (unless I didn't find even a single waypoint...)
  if(completed < 1.0)
    ROS_ERROR_STREAM("trajectory_utils::computeTrajectoryFromWPs : error in computing trajectory >> completed part = " << completed*100.0 << "%");
  if(i < 1)
    return completed;
  
  // interpolate them
  //NOTE: robot_traj, built on robot_model, contains the full robot; trajectory, instead, is only for the group joints
  robot_trajectory::RobotTrajectory robot_traj(ikCheck.get_robot_state().getRobotModel(),group_name);
  robot_traj.setRobotTrajectoryMsg(ikCheck.get_robot_state(),trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  // interpolate trajectory
#if INTERPOLATING_WPs_NR>1
  trajectory.joint_trajectory.points.clear();
  
  for(int j=0; j<rs_vec.size()-1; j++)
  {
    moveit::core::RobotState rs_start(*rs_vec.at(j));
    moveit::core::RobotState rs_end(*rs_vec.at(j+1));
    moveit::core::RobotState rs_wp(rs_start);
    for(int j2=0; j2<INTERPOLATING_WPs_NR; j2++)
    {
      rs_start.interpolate(rs_end,j2/INTERPOLATING_WPs_NR,rs_wp);
      
      moveit::core::RobotStatePtr rs_ptr(new moveit::core::RobotState(rs_wp));
      if(!add_wp_to_traj(rs_ptr,group_name,trajectory))
	ROS_ERROR_STREAM(__func__ << " : unable to add a waypoint to the trajectory being computed...");
    }
  }
#endif

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

bool add_wp_to_traj(const moveit::core::RobotStatePtr& rs, std::string group_name, moveit_msgs::RobotTrajectory& traj)
{
  //NOTE: robot_traj, built on robot_model, contains the full robot; trajectory, instead, is only for the group joints
  robot_trajectory::RobotTrajectory robot_traj(rs->getRobotModel(),group_name); // rs->getJointModelGroup(group_name)->getName());
  if(!traj.joint_trajectory.points.empty())
    robot_traj.setRobotTrajectoryMsg(*rs,traj);
  // NOTE: on purpose, very long time interval to be safe in case something goes wrong!
  robot_traj.addSuffixWayPoint(rs,10.0);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  // compute time stamps
  bool timestamps_ok = iptp.computeTimeStamps(robot_traj);
  if (!timestamps_ok)
    ROS_ERROR("trajectory_utils::add_robot_state_to_traj : unable to compute time stamps for the trajectory");
  else
    robot_traj.getRobotTrajectoryMsg(traj);

  return timestamps_ok;
}

bool append_trajectories(const moveit::core::RobotStatePtr& rs, std::string group_name, moveit_msgs::RobotTrajectory& trajA, moveit_msgs::RobotTrajectory& trajB)
{
  //NOTE: robot_traj, built on robot_model, contains the full robot; trajectory, instead, is only for the group joints
  robot_trajectory::RobotTrajectory robot_traj(rs->getRobotModel(),group_name); // rs->getJointModelGroup(group_name)->getName());
  moveit_msgs::RobotTrajectory tmp_traj = trajA;
  tmp_traj.joint_trajectory.points.insert(tmp_traj.joint_trajectory.points.end(),trajB.joint_trajectory.points.begin(),trajB.joint_trajectory.points.end());
  
  if(!tmp_traj.joint_trajectory.points.empty())
    robot_traj.setRobotTrajectoryMsg(*rs,tmp_traj);
  
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  // compute time stamps
  bool timestamps_ok = iptp.computeTimeStamps(robot_traj);
  if (!timestamps_ok)
    ROS_ERROR("trajectory_utils::add_robot_state_to_traj : unable to compute time stamps for the trajectory");
  else
    robot_traj.getRobotTrajectoryMsg(trajA);

  return timestamps_ok;
}

bool check_trajectory_continuity(moveit_msgs::RobotTrajectory& traj, double allowed_joint_jump, bool check_all_traj)
{
  int joint_nr = traj.joint_trajectory.joint_names.size();
  double max_dist = -1.0;
  int max_i, max_j;
  for(int i=0; i<traj.joint_trajectory.points.size()-1; i++)
  {
    trajectory_msgs::JointTrajectoryPoint& pt1 = traj.joint_trajectory.points.at(i);
    trajectory_msgs::JointTrajectoryPoint& pt2 = traj.joint_trajectory.points.at(i+1);
    for(int j=0; j<joint_nr; j++)
    {
      double diff = std::abs(pt1.positions.at(j) - pt2.positions.at(j));
      if(diff > max_dist)
      {
	max_dist = diff;
	max_i = i;
	max_j = j;
      }
    }
    // if I don't want to check the whole trajectory, I stop at the first point which does not satisfy my bound
    if(!check_all_traj && (max_dist > allowed_joint_jump))
      break;
  }
  
  if(max_dist > allowed_joint_jump)
  {
    ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : maximum allowed distance was " << allowed_joint_jump << "; found instead " << max_dist << " in " << max_i << "-th point at " << max_j << "-th joint, " << (check_all_traj?"":"NOT ") << "checking the trajectory till the end...");
    return false;
  }
  return true;
}
