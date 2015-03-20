#ifndef IK_CONTROL_H
#define IK_CONTROL_H

#include <dual_manipulation_shared/ik_service.h>
#include <dual_manipulation_shared/scene_object_service.h>
#include <dual_manipulation_shared/databasemapper.h>
#include "scene_object_manager.h"
#include "ik_check_capability.h"
#include <std_msgs/String.h>
#include <thread>
#include <XmlRpcValue.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

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
    ikCheckCapability ik_check_capability_;

    // MoveIt! variables
    std::map<std::string,move_group_interface::MoveGroup*> moveGroups_;
    std::map<std::string,moveit::planning_interface::MoveGroup::Plan> movePlans_;
  
    // ros variables
    ros::NodeHandle node;
    std::map<std::string,std::map<std::string,ros::Publisher>> hand_pub;
    std::map<std::string,ros::Publisher> traj_pub_;
    std::map<std::string,ros::Publisher> hand_synergy_pub_;
    std_msgs::String msg;
    
    // utility variables
    std::vector<std::thread*> used_threads_;
    std::map<std::string,bool> busy;
    std::map<std::string,std::string> group_map_;
    std::vector<std::string> chain_names_list_;
    std::vector<std::string> tree_names_list_;
    std::map<std::string,std::vector<std::string>> tree_composition_;
    std::map<std::string,std::string> controller_map_;
    std::map<std::string,std::string> hand_actuated_joint_;
    std::map<std::string,std::vector<std::string>> allowed_collision_prefixes_;
    std::map<std::string,std::vector<std::string>> allowed_collisions_;
    std::map<std::string,std::string> capabilities_;
    std::map<std::string,std::string> traj_pub_topics_;
    std::map<std::string,std::string> hand_synergy_pub_topics_;
    
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
     * @return void
     */
    void planning_thread(dual_manipulation_shared::ik_service::Request req);
    
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
    void simple_homing(std::string ee_name);
    
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
     * 
     */
    inline void stop(){ for(auto item:moveGroups_) item.second->stop(); free_all();}
    
    /**
     * @brief clear all current busy flags
     * 
     */
    inline void free_all(){ for(auto& item:busy) item.second = false;}
    
    /**
     * @brief thread waiting on robot joint state to reach the desired position
     * 
     * @param ee_name
     *    end-effector name
     */
    bool waitForExecution(std::string ee_name);
    
    /**
     * @brief thread waiting on hand joint state to reach the desired position
     * 
     * @param ee_name
     *    end-effector name
     * @param grasp_traj
     *    end-effector grasp trajectory
     */
    bool waitForHandMoved(std::string& hand, double hand_target);

};

}
}

#endif // IK_CONTROL_H
