#ifndef SHARED_IK_MEMORY_H_
#define SHARED_IK_MEMORY_H_

#include <mutex>
#include <dual_manipulation_ik_control/group_structure_manager.h>
#include <dual_manipulation_ik_control/robot_controller_interface.h>
#include <dual_manipulation_ik_control/robot_state_manager.h>
#include <dual_manipulation_ik_control/scene_object_manager.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <XmlRpcValue.h>

namespace dual_manipulation
{
namespace ik_control
{

/**
 * @brief A structure to share resources across implemented capabilities: plan, move, grasp, ...
 */
class shared_ik_memory
{
public:
    shared_ik_memory(XmlRpc::XmlRpcValue& params, ros::NodeHandle& nh);
    ~shared_ik_memory() {}
    
    /**
     * @brief Reset internal variables
     */
    void reset();
    
    /**
     * @brief Atomically get/set the planned trajectory for the specified group.
     * 
     * @param group The name of the group to use for swapping the trajectory
     * @param traj The trajectory to use for swapping
     * 
     * @return False if the group did not exist, true otherwise (but @p traj can be empty)
     */
    bool swapTrajectory(const std::string& group, moveit_msgs::RobotTrajectory& traj);
    
    /**
     * @brief Inform that a trajectory execution is pending and planning should not be initialized until it started. To query this property, call @fn getNextTrajectoyEndTime
     * 
     * @return false if a trajectory execution was already pending, true otherwise.
     */
    bool setPendingTrajectoryExecution();
    
    /**
     * @brief Set the (relative) duration of the next trajectory execution, to give more time to planning. Can be called only if a trajectory execution is pending.
     * 
     * @return false if a trajectory execution was not pending, true otherwise.
     */
    bool setNextTrajectoryRelativeEndTime(const ros::Duration& dt);
    
    /**
     * @brief Get trajectory execution end time. Planning should not be initialized until there is no trajectory execution pending.
     * 
     * @param end_t The time at which the trajectory execution is supposed to end
     * 
     * @return false if a trajectory execution is still pending, true otherwise
     */
    bool getNextTrajectoyEndTime(ros::Time& end_t);
    
    /**
     * @brief Get a copy of the robot state which can be used for planning
     */
    moveit::core::RobotState getPlanningRobotState();
    
    /**
     * @brief Reset the joint group @p group in the robot state used for planning
     * 
     * @param group The group to reset in the planning robot state
     * 
     * @return true on success
     */
    bool resetPlanningRobotState(const std::string& group);
    
    /**
     * @brief Reset the joint group @p group in the robot state used for planning using the last waypoint in the trajectory @p traj
     * 
     * @param group The group to reset in the planning robot state
     * @param traj The trajectory to use in resetting the robot state
     * 
     * @return true on success
     */
    bool resetPlanningRobotState(const std::string& group, const moveit_msgs::RobotTrajectory& traj);
    
public:
    std::mutex m;
    XmlRpc::XmlRpcValue* ik_control_params;
    // manage robot group structure
    std::unique_ptr<const GroupStructureManager> groupManager;
    // manage robot controllers
    std::unique_ptr<const RobotControllerInterface> robotController;
    // manage robot states
    std::unique_ptr<const RobotStateManager> robotStateManager;
    // managing the objects in the scene
    std::unique_ptr<SceneObjectManager> sceneObjectManager;
    
private:
    // MoveIt! variables
    moveit::core::RobotModelPtr robot_model_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    // utility variables
    std::string joint_states_;
    std::string robot_description_;
    std::string full_robot_group_;
    
    // share the robot state to use for next planning
    std::mutex robotState_mutex_;
    moveit::core::RobotStatePtr planning_init_rs_;
    // trajectory execution expected end-time
    std::mutex end_time_mutex_;
    ros::Time movement_end_time_;
    // share the motion plans among planning/control capabilities
    std::mutex movePlans_mutex_;
    std::map<std::string,moveit::planning_interface::MoveGroup::Plan> movePlans_;
    
private:
    /**
     * @brief Utility function to parse parameters from the parameter server
     * 
     * @param params params got from the parameter server
     */
    void parseParameters(XmlRpc::XmlRpcValue& params);
};

}
}

#endif // SHARED_IK_MEMORY_H_
