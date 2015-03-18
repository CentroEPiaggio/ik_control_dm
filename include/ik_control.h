#ifndef IK_CONTROL_H
#define IK_CONTROL_H

#include <dual_manipulation_shared/ik_service.h>
#include <dual_manipulation_shared/scene_object_service.h>
#include <dual_manipulation_shared/databasemapper.h>
#include "scene_object_manager.h"
#include <std_msgs/String.h>
#include <thread>
#include <XmlRpcValue.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPositionIK.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

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
    // initialization variable - for possible future usage
    bool isInitialized_;
    bool kinematics_only_ = false;
    std::vector<std::thread*> used_threads_;
    
    // managing the objects in the scene
    sceneObjectManager scene_object_manager_;

    // MoveIt! variables
    robot_state::RobotStatePtr robot_state_;
    std::map<std::string,move_group_interface::MoveGroup*> moveGroups_;
    std::map<std::string,kdl_kinematics_plugin::KDLKinematicsPlugin*> kinematics_plugin_;
    std::map<std::string,moveit::planning_interface::MoveGroup::Plan> movePlans_;
  
    std::map<std::string,bool> busy;
    ros::NodeHandle node;
    std::map<std::string,std::map<std::string,ros::Publisher>> hand_pub;
    std_msgs::String msg;
    std::map<std::string,std::string> group_map_;
    std::map<std::string,std::string> controller_map_;
    std::map<std::string,std::string> ee_map_,hand_actuated_joint_;
    std::map<std::string,std::vector<std::string>> allowed_collisions_;
    
    std::map<std::string,ros::Publisher> traj_pub_;
    std::map<std::string,ros::Publisher> hand_synergy_pub_;
    moveit_msgs::PlanningScene planning_scene_;
    
    ros::ServiceClient ik_serviceClient_;
    XmlRpc::XmlRpcValue ik_control_params;
    
    double position_threshold=0;
    double velocity_threshold=0;
    double hand_max_velocity=0;
    double hand_position_threshold=0;
    
    void parseParameters(XmlRpc::XmlRpcValue& params);
    
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
     * @brief utility function to split a full robot trajectory to single arm trajectories
     * 
     * @return bool
     */
    bool splitFullRobotPlan();
    
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
     * @brief utility function to convert a waypoint sequence into a robot trajectory
     */
    bool computeTrajectoryFromWPs(moveit_msgs::RobotTrajectory& trajectory,const dual_manipulation_shared::ik_service::Request& req);
    
    /**
     * @brief utility function to associate hand timing to the robot timing
     * 
     * This function aligns hand timing w.r.t. robot timing, and checks for hand velocity limits, slowing down the robot trajectory if needed
     * TODO: implement this last part
     */
    bool computeHandTiming(moveit_msgs::RobotTrajectory& trajectory,dual_manipulation_shared::ik_service::Request& req);
    
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
