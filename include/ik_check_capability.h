#ifndef IKCHECKCAPABILITY_H
#define IKCHECKCAPABILITY_H

#include <ros/ros.h>
#include <XmlRpcValue.h>

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
    std::vector<std::string> chain_names_list_;
    std::vector<std::string> tree_names_list_;
    std::map<std::string,std::vector<std::string>> tree_composition_;
    
    // managing external parameters
    XmlRpc::XmlRpcValue ik_control_params;
    
//     std::map<std::string,kdl_kinematics_plugin::KDLKinematicsPlugin*> kinematics_plugin_;
//     moveit_msgs::PlanningScene planning_scene_;
    
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
};

}
}

#endif // IKCHECKCAPABILITY_H
