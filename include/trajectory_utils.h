#ifndef TRAJECTORY_UTILS_H
#define TRAJECTORY_UTILS_H

#include "ros/ros.h"
#include <dual_manipulation_shared/ik_service.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "ik_check_capability/ik_check_capability.h"

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
  * @brief utility function to convert a waypoint sequence into a robot trajectory
  * 
  * @param trajectory
  *   contains the returned moveit_msgs::RobotTrajectory if everything worked
  * @param waypoints
  *   points to follow
  * @param ikCheck
  *   an ikCheckCapability instance which can operate on the group we want to compute WPs for
  * @param group_name
  *   the name of the group we want to operate on (as defined in the SRDF)
  * @param ee_name
  *   the name of the group we want to operate on (as defined in ikCheckCapability)
  * @param avoid_collisions
  *   default: false - says whether to fail if a collision is found
  * 
  * @return completed percentage of the trajectory, between 0 and 1; -1 on failure of the time parametrization
  */
double computeTrajectoryFromWPs(moveit_msgs::RobotTrajectory& trajectory, const std::vector <geometry_msgs::Pose >& waypoints, dual_manipulation::ik_control::ikCheckCapability& ikCheck, std::string group_name, std::string ee_name, bool avoid_collisions = false);

/**
  * @brief utility function to associate hand timing to the robot timing
  * 
  * This function aligns hand timing w.r.t. robot timing, and checks for hand velocity limits, slowing down the robot trajectory if needed
  * TODO: implement this last part
  */
bool computeHandTiming(moveit_msgs::RobotTrajectory& trajectory,dual_manipulation_shared::ik_service::Request& req);

/**
  * @brief utility to add a waypoint to a trajectory message using a robot state
  * 
  * @param rs the robot state representing the waypoint
  * @param group_name the name of the group to which the trajectory is associated
  * @param traj robot trajectory message to which a waypoint is added
  */
bool add_wp_to_traj(const moveit::core::RobotStatePtr& rs, std::string group_name, moveit_msgs::RobotTrajectory& traj);

/**
  * @brief utility to append a trajectory message to another (for the same group)
  * 
  * @param rs a robot state associated to the group of which we want to append trajectories
  * @param group_name the name of the group to which the trajectory is associated
  * @param trajA robot trajectory message to which a suffix trajectory is added
  * @param trajB robot trajectory message to be added
  */
bool append_trajectories(const moveit::core::RobotStatePtr& rs, std::string group_name, moveit_msgs::RobotTrajectory& trajA, moveit_msgs::RobotTrajectory& trajB);

#endif //TRAJECTORY_UTILS_H
