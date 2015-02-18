#ifndef IK_CONTROL_H
#define IK_CONTROL_H

#include <dual_manipulation_shared/ik_service.h>
#include <dual_manipulation_shared/scene_object_service.h>
#include "../../shared/include/dual_manipulation_shared/databasemapper.h"
#include <std_msgs/String.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

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
    std::map<std::string,std::string> ee_map_;
    std::map<std::string,std::vector<std::string>> allowed_collisions_;
    
    ros::Publisher robot_state_publisher_;

    std::map<std::string,moveit_msgs::AttachedCollisionObject> grasped_objects_map_;
    std::map<std::string,moveit_msgs::AttachedCollisionObject> world_objects_map_;
    ros::Publisher collision_object_publisher_,attached_collision_object_publisher_;
    std::map<std::string,ros::Publisher> traj_pub_;
    std::map<std::string,ros::Publisher> hand_synergy_pub_;
    moveit_msgs::PlanningScene planning_scene_;
    
    databaseMapper db_mapper_;
    
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
     * @brief utility function to load a mesh and attach it to a collision object
     * 
     * @param attObject
     *   the attached collision object to which the mesh has to be attached
     *   note that the weight is interpreted as the index of the object in the DB
     *   TODO: something better
     * @return void
     */
    void loadAndAttachMesh(moveit_msgs::AttachedCollisionObject &attObject);
    
    /**
     * @brief insert a new object in the planning scene
     * 
     * @param req
     *   the same req from @e scene_object_service
     * @return bool
     */
    bool addObject(dual_manipulation_shared::scene_object_service::Request req);
    
    /**
     * @brief remove an object from the planning scene
     * 
     * @param object_id
     *   the id of the object to be removed from the scene
     * @return bool
     */
    bool removeObject(std::string &object_id);
    
    /**
     * @brief an object present in the planning scene becomes attached to a robot link
     * 
     * @param req
     *   the the same req from @e scene_object_service
     * @return bool
     */
    bool attachObject(dual_manipulation_shared::scene_object_service::Request& req);

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
     * @brief function to move a group to its home position (does not change hand opening)
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

};

}
}

#endif // IK_CONTROL_H
