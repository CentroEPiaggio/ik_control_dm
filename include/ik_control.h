#ifndef IK_CONTROL_H
#define IK_CONTROL_H

#include <dual_manipulation_shared/ik_service.h>
#include <dual_manipulation_shared/scene_object_service.h>
#include "scene_object_manager.h"
#include "ik_check_capability/ik_check_capability.h"
#include <thread>
#include <XmlRpcValue.h>
#include <mutex>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// capabilities definition
#include <dual_manipulation_shared/ik_control_capabilities.h>

namespace dual_manipulation
{
namespace ik_control
{
  
/**
  * @brief This is a class that is used from the ros_server to perform a desired ik using a dedicated thread (one for each end effector).
  * 
  */
class ikControl
{
public:
    ikControl();
    ~ikControl();
    
    /**
     * @brief interface function to perform the @e ik_service
     * 
     * @param req
     *   the req from the @e ik_service
     * @return bool
     */
    bool perform_ik(dual_manipulation_shared::ik_service::Request &req);
    
    /**
     * @brief interface function to manage objects
     * 
     * @param req
     *   the req from the @e scene_object_service
     * @return bool
     */
    bool manage_object(dual_manipulation_shared::scene_object_service::Request &req);
    
private:
    // managing the objects in the scene
    sceneObjectManager scene_object_manager_;
    // manage IK requests
    ikCheckCapability *ik_check_legacy_;
    // internal usage IK
    ikCheckCapability *ik_check_;

    // MoveIt! variables
    std::map<std::string,move_group_interface::MoveGroup*> moveGroups_;
    std::map<std::string,moveit::planning_interface::MoveGroup::Plan> movePlans_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr target_rs_, planning_init_rs_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  
    // ros variables
    ros::NodeHandle node;
    std::map<ik_control_capabilities,std::map<std::string,ros::Publisher>> hand_pub;
    std::map<std::string,ros::Publisher> traj_pub_;
    std::map<std::string,ros::Publisher> hand_synergy_pub_;
    
    // utility variables
    std::vector<std::thread*> used_threads_;
    std::map<ik_control_capability_types,std::map<std::string,bool>> busy;
    ik_control_capability capabilities_;
    std::map<std::string,std::string> group_map_;
    std::vector<std::string> chain_names_list_;
    std::vector<std::string> tree_names_list_;
    std::map<std::string,std::vector<std::string>> tree_composition_;
    std::map<std::string,std::string> controller_map_;
    std::map<std::string,std::string> hand_actuated_joint_;
    std::map<std::string,std::vector<std::string>> allowed_collision_prefixes_;
    std::map<std::string,std::vector<std::string>> allowed_collisions_;
    std::map<std::string,std::string> traj_pub_topics_;
    std::map<std::string,std::string> hand_synergy_pub_topics_;
    std::map<std::string,std::string> grasped_obj_map_;
    std::mutex map_mutex_;
    std::mutex hand_synergy_pub_mutex_;
    std::mutex scene_object_mutex_;
    std::mutex moveGroups_mutex_;
    std::mutex movePlans_mutex_;
    std::mutex robotState_mutex_;
    std::mutex ikCheck_mutex_;
    
    // planner parameters
    std::string planner_id_;
    double planning_time_;
    double goal_position_tolerance_;
    double goal_orientation_tolerance_;
    double goal_joint_tolerance_;
    std::vector<double> ws_bounds_;
    
    // managing external parameters
    XmlRpc::XmlRpcValue ik_control_params;
    
    bool kinematics_only_;      // if false (default), wait for the controller
    double position_threshold;  // threshold on square sum : avg is 0.01 rad on each joint
    double velocity_threshold;  // threshold on square sum : avg is 0.01 rad/s on each joint
    double hand_max_velocity;   // maximum hand velocity : avg is 2.0, closes completely [0.0->1.0] in half a second
    double hand_position_threshold; // threshold on hand position to consider a desired one reached
    double clik_threshold_;     // minimum value allowed to a CLIK solution to be considered valid
    
    /**
     * @brief utility function to parse parameters from the parameter server
     * 
     * @param params
     *   all useful params got from the parameter server
     * @return void
     */
    void parseParameters(XmlRpc::XmlRpcValue& params);
    
    /**
     * @brief utility function to set all class parameters to their default value
     * 
     * @return void
     */
    void setDefaultParameters();

    /**
     * @brief utility function to set class variables which depend on parameters
     * 
     * @return void
     */
    void setParameterDependentVariables();

    /**
     * @brief this is the thread body to perform IK feasibility check (no collision considered)
     * 
     * @param req
     *   the same req from the @e ik_service
     * @return void
     */
    void ik_check_thread(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief this is the thread body to perform trajectory generation
     * 
     * @param req
     *   the same req from the @e ik_service
     * @param check_collisions flag to say whether to check for collisions
     * @param use_clik allow for close (in cartesian-space) solutions using Closed-Loop Inverse-Kinematics (CLIK)
     * 
     * @return void
     */
    void planning_thread(dual_manipulation_shared::ik_service::Request req, bool check_collisions, bool use_clik);
    
    /**
     * @brief execute last planned path
     * 
     * @return void
     */
    void execute_plan(dual_manipulation_shared::ik_service::Request req);
    
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
    bool moveHand(std::string &hand, std::vector<double> &q, std::vector<double> &t);
    
    /**
     * @brief function to move the hand to the desired configuration with the desired timing
     * 
     * @param hand
     *   name of the hand to move
     * @param grasp_traj
     *   joint trajectory of the hand
     * @return bool: true for grasp success (at now, true by default)
     */
    bool moveHand(std::string &hand, trajectory_msgs::JointTrajectory& grasp_traj);
    
    /**
     * @brief function to move a group to its home position and to open the hand
     * 
     * @param ee_name
     *   which end-effector bring back home
     * @return void
     */
    void simple_homing(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief handler function for grasping an object
     * 
     * @param req
     *   the same req from the @e ik_service
     * @return void
     */
    void grasp(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief handler function for ungrasping an object
     * 
     * @param req
     *   the same req from the @e ik_service
     * @return void
     */
    void ungrasp(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief stop the current execution of a trajectory (if any)
     * Stop the current trajectory being executed, free all capabilities, and reset the planning initial state to the current robot state
     * 
     */
    inline void stop(){ for(auto item:moveGroups_) item.second->stop(); free_all();}
    
    /**
     * @brief clear all current busy flags
     * 
     */
    inline void free_all(){ map_mutex_.lock(); for(auto& item:busy) for(auto& item2:item.second) item2.second = false; map_mutex_.unlock(); reset();}
    
    /**
     * @brief resets all robot states and movePlans
     * 
     */
    inline void reset(){ 
      robotState_mutex_.lock();
      reset_robot_state(planning_init_rs_); reset_robot_state(target_rs_);
      robotState_mutex_.unlock();
      movePlans_mutex_.lock();
      for(auto& plan:movePlans_){ move_group_interface::MoveGroup::Plan tmp_plan; std::swap(plan.second,tmp_plan);}
      movePlans_mutex_.unlock();
    }
    
    /**
     * @brief thread waiting on robot joint state to reach the desired position
     * 
     * @param ee_name
     *    end-effector name
     * @param traj
     *    trajectory we have to wait the execution of
     * 
     * @return bool
     */
    bool waitForExecution(std::string ee_name, moveit_msgs::RobotTrajectory traj);
    
    /**
     * @brief thread waiting on hand joint state to reach the desired position
     * 
     * @param ee_name
     *    end-effector name
     * @param grasp_traj
     *    end-effector grasp trajectory
     */
    bool waitForHandMoved(std::string& hand, double hand_target);
    
    /**
     * @brief function to check whether a capability is busy, and to lock it in case it is
     * 
     * @param ee_name
     *    end-effector name
     * @param capability
     *    capability to check
     */
    bool is_free_make_busy(std::string ee_name, std::string capability);
    
    /**
     * @brief utility to reset the state of the parameter @p rs to the current robot state
     * 
     * @param rs the robot state to reset
     * 
     * @return true on success
     */
    bool reset_robot_state(const moveit::core::RobotStatePtr& rs);
    
    /**
     * @brief utility to reset the state of the parameter @p rs to the final position in the @p traj trajectory
     * 
     * @param rs the robot state to reset
     * @param ee_name the end-effector name to which the trajectory is associated
     * @param traj robot trajectory of which to use the last waypoint to update @p rs
     * 
     * @return true on success
     */
    bool reset_robot_state(const moveit::core::RobotStatePtr& rs, std::string ee_name, const moveit_msgs::RobotTrajectory& traj);
    
    /**
     * @brief set the target robot state of the end-effector @p ee_name to the target specified in the SRDF with name @p named_target
     * 
     * @param ee_name the end-effector we want to set a target for
     * @param named_target the target name as specified in the SRDF
     * 
     * @return true on success
     */
    bool set_target(std::string ee_name, std::string named_target);
    
    /**
     * @brief set the target robot state of the end-effector @p ee_name to the target pose(s)
     * 
     * @param ee_name the end-effector we want to set a target for
     * @param ee_poses the target pose(s), one for each end-effector
     * @param check_collisions flag to say whether to check for collisions
     * @param use_clik allow for close (in cartesian-space) solutions using Closed-Loop Inverse-Kinematics (CLIK)
     * 
     * @return true on success
     */
    bool set_target(std::string ee_name, std::vector<geometry_msgs::Pose> ee_poses, bool check_collisions, bool use_clik);
    
    /**
     * @brief set the target robot state of the end-effector @p ee_name to the target pose(s)
     * 
     * @param ee_name the end-effector we want to set a target for
     * @param ee_poses the target pose(s), one for each end-effector
     * 
     * @return true on success
     */
    bool set_close_target(std::string ee_name, std::vector<geometry_msgs::Pose> ee_poses, unsigned int trials_nr = 1, bool check_collisions = true, bool return_approximate_solution = false, double allowed_distance = 1000.0);
};

}
}

#endif // IK_CONTROL_H
