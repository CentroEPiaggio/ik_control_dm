#ifndef ROBOT_STATE_MANAGER_H
#define ROBOT_STATE_MANAGER_H

#include <map>
#include <mutex>
#include <XmlRpcValue.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>

/**
 * @brief A class to manage robot states
 * 
 * This class allows to manage robot states, making it easy to update a state from the published robot state or other states/trajectories.
 */
class RobotStateManager
{
public:
    /**
     * @brief Initialize a robot state manager
     * 
     * @param robot_model A robot model to construct the state monitor
     * @param joint_states The topic to use for listening to joint states
     */
    RobotStateManager(const moveit::core::RobotModelConstPtr& robot_model, const std::string& joint_states);
    ~RobotStateManager() {}
    
    /**
     * @brief utility to reset the state of the parameter @p rs to the current robot state
     * 
     * @param rs the robot state to reset
     * @param group the SRDF group to reset
     * @param rs_mutex the robot_state mutex to use to access the robot state
     * 
     * @return true on success
     */
    bool reset_robot_state(const moveit::core::RobotStatePtr& rs, const std::string& group, std::mutex& rs_mutex) const;
    
    /**
     * @brief utility to reset the state of the parameter @p rs to the final position in the @p traj trajectory
     * 
     * @param rs the robot state to reset
     * @param group the SRDF group to reset
     * @param rs_mutex the robot_state mutex to use to access the robot state
     * @param traj robot trajectory of which to use the last waypoint to update @p rs
     * 
     * @return true on success
     */
    bool reset_robot_state(const moveit::core::RobotStatePtr& rs, const std::string& group, std::mutex& rs_mutex, const moveit_msgs::RobotTrajectory& traj) const;
    
private:
    mutable robot_state::RobotStatePtr current_state_;
    planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
    mutable std::mutex cs_mutex_;
    
private:
    
    /**
     * @brief Read updated robot state from server.
     * 
     * @param group the SRDF group to read the state of
     * @param wait_seconds time to wait (at most) for the updated state
     * 
     * @return True on success
     */
    bool updateCurrentState(const std::string& group, double wait_seconds = 1.0) const;
};

#endif // ROBOT_STATE_MANAGER_H
