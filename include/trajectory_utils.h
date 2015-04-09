#ifndef TRAJECTORY_UTILS_H
#define TRAJECTORY_UTILS_H

#include "ros/ros.h"
#include <dual_manipulation_shared/ik_service.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

/**
  * @brief utility function to split a full robot trajectory to single arm trajectories
  * 
  * @return bool
  * 
  * TODO: make this function more general, with a signature like:
  * splitMultiChainTrajectory(robot_trajectory::RobotTrajectory& full_traj, const std::vector<robot_trajectory::RobotTrajectory>& single_chain_traj)
  * only joint names should be present (or will be considered) in single_chain_traj array
  */
bool splitFullRobotPlan(std::map<std::string,move_group_interface::MoveGroup*> moveGroups_, std::map<std::string,moveit::planning_interface::MoveGroup::Plan> movePlans_);

/**
  * @brief utility function to convert a waypoint sequence into a robot trajectory
  * 
  * @param trajectory
  *   contains the returned moveit_msgs::RobotTrajectory if everything worked
  * @param waypoints
  *   points to follow
  * @param moveGroup
  *   group to use for generating the trajectory
  * @param avoid_collisions
  *   default: false - say whether to fail if a collision is found (a posteriori)
  * @param eef_step
  *   default 1.0 [m] - if the distance between consecutive waypoints is higher than @p eef_step, interpolate on a straight line
  *   the default is relatively high, meaning: do not interpolate
  * @param jump_threshold
  *   default 0.0 (don't check) - if not zero, if joints in consecutive waypoints are further away than @p jump_threshold (* average distance), the trajectory is considered wrong and truncated right before the failing waypoint
  * 
  * @return completed percentage of the trajectory, between 0 and 1; -1 on failure of the time parametrization
  */
double computeTrajectoryFromWPs(moveit_msgs::RobotTrajectory& trajectory, const std::vector <geometry_msgs::Pose >& waypoints, moveit::planning_interface::MoveGroup* moveGroup, bool avoid_collisions = false, double eef_step = 1.0, double jump_threshold = 0.0);

/**
  * @brief utility function to associate hand timing to the robot timing
  * 
  * This function aligns hand timing w.r.t. robot timing, and checks for hand velocity limits, slowing down the robot trajectory if needed
  * TODO: implement this last part
  */
bool computeHandTiming(moveit_msgs::RobotTrajectory& trajectory,dual_manipulation_shared::ik_service::Request& req);

#endif //TRAJECTORY_UTILS_H
