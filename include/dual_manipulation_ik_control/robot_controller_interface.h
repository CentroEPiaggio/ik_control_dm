#ifndef ROBOT_CONTROLLER_INTERFACE_H
#define ROBOT_CONTROLLER_INTERFACE_H

#include <XmlRpcValue.h>
#include <mutex>
#include <dual_manipulation_ik_control/group_structure_manager.h>
#include <dual_manipulation_ik_control/robot_state_manager.h>
#include <atomic>
#include <std_msgs/String.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

namespace dual_manipulation
{
namespace ik_control
{

/**
 * @brief Interface class to command the robot: implements blocking and non-blocking trajectory following controllers.
 */
class RobotControllerInterface
{
public:
    RobotControllerInterface(XmlRpc::XmlRpcValue& params, const GroupStructureManager& groupManager_, const std::string& joint_states, const RobotStateManager& rsManager_, const ros::NodeHandle& node_ = ros::NodeHandle());
    ~RobotControllerInterface() {}
    
    void resetRobotModel(moveit::core::RobotModelConstPtr robot_model);
    
    /**
     * @brief execute plan without waiting
     */
    moveit::planning_interface::MoveItErrorCode asyncExecute(const moveit::planning_interface::MoveGroup::Plan& plan);
    
    /**
     * @brief function to move the hand to the desired configuration with the desired timing
     * 
     * @param hand
     *   name of the hand to move
     * @param q
     *   array of synergy joint position of the hand
     * @param t
     *   timing vector (must be of the same length of q, otherwise a default timing is used)
     * @return bool: true for grasp success (at now, true by default)
     */
    bool moveHand(const std::string& hand, const std::vector< double >& q, std::vector< double >& t, trajectory_msgs::JointTrajectory& grasp_traj);
    
    /**
     * @brief function to move the hand to the desired configuration with the desired timing
     * 
     * @param hand
     *   name of the hand to move
     * @param grasp_traj
     *   joint trajectory of the hand
     * @return bool: true for grasp success (at now, true by default)
     */
    bool moveHand(const std::string& hand, const trajectory_msgs::JointTrajectory& grasp_traj);
    
    /**
     * @brief stop the current execution of a trajectory (if any)
     * Stop the current trajectory being executed, free all capabilities, and reset the planning initial state to the current robot state
     */
    inline void stop()
    {
        std_msgs::String event;
        event.data = "stop";
        trajectory_event_publisher_.publish(event);
    }
    
    /**
     * @brief blocking function to wait on robot joint state to reach the desired position
     * 
     * @param ee_name
     *    end-effector name
     * @param traj
     *    trajectory we have to wait the execution of
     * 
     * @return bool
     * 
     * @post if the trajectory to wait for results in an error, the robot_state to use for planning may end up inconsistent and must be reset outside
     */
    bool waitForExecution(std::string ee_name, moveit_msgs::RobotTrajectory traj);
    
    /**
     * @brief blocking function to wait on hand joint state to reach the desired position
     * 
     * @param ee_name
     *    end-effector name
     * @param grasp_traj
     *    end-effector grasp trajectory
     * 
     * @return true on success
     * 
     * @post if the trajectory to wait for results in an error, the robot_state to use for planning may end up inconsistent and must be reset outside
     */
    bool waitForHandMoved(std::string& hand, double hand_target, const trajectory_msgs::JointTrajectory& traj);
    
private:
    std::atomic_bool busy;
    bool initialized;
    
    // ros variables
    ros::NodeHandle node;
    std::map<std::string,ros::Publisher> hand_synergy_pub_;
    ros::Publisher trajectory_event_publisher_;
    
    const GroupStructureManager& groupManager;
    const RobotStateManager& rsManager;
    
    // MoveIt! variables
    std::map<std::string,move_group_interface::MoveGroup*> moveGroups_;
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::core::RobotStatePtr visual_rs_;

    // utility variables
    std::map<std::string,std::string> controller_map_;
    std::map<std::string,std::string> hand_actuated_joint_;
    std::map<std::string,std::string> hand_synergy_pub_topics_;
    std::mutex map_mutex_; // controller_map_, hand_actuated_joint_
    std::mutex hand_synergy_pub_mutex_;
    std::mutex moveGroup_mutex_;
    const std::string joint_states_;

    bool kinematics_only_;      // if false (default), wait for the controller
    double position_threshold;  // threshold on square sum : avg is 0.01 rad on each joint
    double velocity_threshold;  // threshold on square sum : avg is 0.01 rad/s on each joint
    double hand_position_threshold; // threshold on hand position to consider a desired one reached
    
private:
    /**
     * @brief utility function to parse parameters from the parameter server
     * 
     * @param params
     *   all useful params got from the parameter server
     * @return void
     */
    void parseParameters(XmlRpc::XmlRpcValue& params);
    
    /**
     * @brief utility function to set class variables which depend on parameters
     * 
     * @return void
     */
    void setParameterDependentVariables();
    
    /**
     * @brief display the trajectory of the robot - works when there is no outside /joint_states publisher, and kinematics_only_ is set to true
     * 
     * @param trajectory_msg the trajectory to be displayed
     */
    bool publishTrajectoryPath(const moveit_msgs::RobotTrajectory& trajectory_msg);
    
    /**
     * @brief Function to reset move groups based on a newly arrived robot model
     */
    void resetMoveGroup();
};

}
}

#endif // ROBOT_CONTROLLER_INTERFACE_H
