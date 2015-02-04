#ifndef IK_CONTROL_H
#define IK_CONTROL_H

#include "dual_manipulation_shared/ik_service.h"
#include <std_msgs/String.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>

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

private:
    // initialization variable - for possible future usage
    bool isInitialized_;

    // MoveIt! variables
    robot_state::RobotStatePtr robot_state_;
    robot_model::JointModelGroup* left_hand_arm_group_;
    robot_model::JointModelGroup* right_hand_arm_group_;
    robot_model::JointModelGroup* full_robot_group_;
    std::map<std::string,move_group_interface::MoveGroup*> moveGroups_;
    std::map<std::string,kdl_kinematics_plugin::KDLKinematicsPlugin*> kinematics_plugin_;
  
    std::map<std::string,bool> busy;
    ros::NodeHandle node;
    std::map<std::string,ros::Publisher> hand_pub;
    std_msgs::String msg;
    std::map<std::string,std::string> group_map_;
    
    ros::Publisher robot_state_publisher_;

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
    
};

}
}

#endif // IK_CONTROL_H
