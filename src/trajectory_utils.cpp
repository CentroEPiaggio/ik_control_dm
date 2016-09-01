#include "trajectory_utils.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "ik_check_capability/ik_check_capability.h"

#define CLASS_NAMESPACE "trajectory_utils::"

double computeTrajectoryFromWPs(moveit_msgs::RobotTrajectory& trajectory, const std::vector< geometry_msgs::Pose >& waypoints, dual_manipulation::ik_control::ikCheckCapability& ikCheck, std::string group_name, std::string ee_name, bool avoid_collisions, double allowed_distance, std::vector<double>& single_distances)
{
  // compute waypoints
  double completed = 0.0;
  int i;
  std::vector<double> solution;
  std::vector<double> initial_guess;
  
  // store starting robot state and add it to the trajectory
  moveit::core::RobotStatePtr init_rs(new moveit::core::RobotState(ikCheck.get_robot_state()));
  add_wp_to_traj(init_rs,group_name,trajectory);
  
  unsigned int trials_nr = 10;
  bool return_approximate_solution = false;
  
  for(i=0; i<waypoints.size(); i++)
  {
    if(!ikCheck.find_closest_group_ik(ee_name,waypoints.at(i),solution,allowed_distance,single_distances,trials_nr,initial_guess,avoid_collisions,return_approximate_solution))
      break;
    
    moveit::core::RobotStatePtr rs(new moveit::core::RobotState(ikCheck.get_robot_state()));
    
    if(!add_wp_to_traj(rs,group_name,trajectory))
      break;
  }
  completed = (double)i/waypoints.size();

    // show an error if I didn't complete the trajectory, but continue with the parametrization (unless I didn't find even a single waypoint...)
    if(i < 1)
    {
        ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : error in computing trajectory >> NO waypoint was converted!");
    }
    if(completed < 1.0)
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : error in computing trajectory >> completed part = " << completed*100.0 << "%");
  
  // interpolate them
  //NOTE: robot_traj, built on robot_model, contains the full robot; trajectory, instead, is only for the group joints
  robot_trajectory::RobotTrajectory robot_traj(ikCheck.get_robot_state().getRobotModel(),group_name);
  robot_traj.setRobotTrajectoryMsg(ikCheck.get_robot_state(),trajectory);
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
