#ifndef IKCHECKCAPABILITY_H
#define IKCHECKCAPABILITY_H

#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
// #include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit_msgs/PlanningScene.h>

#include <dual_manipulation_shared/ik_service.h>

namespace dual_manipulation
{
namespace ik_control
{
  
/**
  * @brief This is a class to manage inverse kinematics checking with different implementations
  * 
  */
class ikCheckCapability
{
public:
    ikCheckCapability();
    ~ikCheckCapability();
    
    /**
     * @brief interface function to manage IK requests
     * 
     * @param req
     *   the req from the @e ik_ros_service
     * @return bool
     */
    bool manage_ik(dual_manipulation_shared::ik_service::Request req);
  
private:
    // ros variables
    ros::NodeHandle node;
    ros::ServiceClient ik_serviceClient_;
  
    // MoveIt! variables
    std::map<std::string,move_group_interface::MoveGroup*> moveGroups_;
    
    // utility variables
    std::map<std::string,std::string> group_map_;
    
//     std::map<std::string,kdl_kinematics_plugin::KDLKinematicsPlugin*> kinematics_plugin_;
//     moveit_msgs::PlanningScene planning_scene_;
    
};

}
}

#endif // IKCHECKCAPABILITY_H
