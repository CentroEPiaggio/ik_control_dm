#ifndef TRAJECTORY_UTILS_H
#define TRAJECTORY_UTILS_H

#include "ros/ros.h"
#include <dual_manipulation_shared/ik_service.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "ik_check_capability/ik_check_capability.h"

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
double computeTrajectoryFromWPs(moveit_msgs::RobotTrajectory& trajectory, const std::vector <geometry_msgs::Pose >& waypoints, dual_manipulation::ik_control::ikCheckCapability& ikCheck, std::string group_name, std::string ee_name, bool avoid_collisions, double allowed_distance, std::vector<double>& single_distances);

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

/**
 * @brief utility to check whether a trajectory has joint values which only differ of at most a certain amount between successive steps
 * 
 * @param traj robot trajectory to be tested
 * @param allowed_joint_jump maximum allowed joint difference between successive samples in the trajectory
 * @param check_all_traj flag to say whether to check the whole trajectory or stop at the first occurrence of violated distance
 */
bool check_trajectory_continuity(moveit_msgs::RobotTrajectory& traj, double allowed_joint_jump, bool check_all_traj = false);

#endif //TRAJECTORY_UTILS_H
