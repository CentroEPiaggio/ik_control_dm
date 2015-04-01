#ifndef IKCHECKCAPABILITY_H
#define IKCHECKCAPABILITY_H

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <mutex>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

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
    ros::Subscriber scene_sub_;
  
    // MoveIt! variables
    std::map<std::string,moveit::core::JointModelGroup*> jm_groups_;
    std::map<std::string,move_group_interface::MoveGroup*> moveGroups_;
    planning_scene::PlanningScenePtr planning_scene_;
    robot_model::RobotModelPtr kinematic_model_;
    robot_state::RobotStatePtr kinematic_state_;
    
    // utility variables
    bool is_initialized_ = false;
    std::map<std::string,std::string> group_map_;
    std::vector<std::string> chain_names_list_;
    std::vector<std::string> tree_names_list_;
    std::vector<std::string> group_names_;
    std::map<std::string,std::vector<std::string>> tree_composition_;
    std::mutex scene_mutex_;
    std::mutex map_mutex_;
    double default_ik_timeout_ = 0.005;
    unsigned int default_ik_attempts_ = 10;
    
    // managing external parameters
    XmlRpc::XmlRpcValue ik_control_params;
    
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
     * @brief callback to update the planning scene using monitored_planning_scene
     * 
     * @return void
     */
    void scene_callback(const moveit_msgs::PlanningScene_< std::allocator< void > >::ConstPtr& plan_msg);
};

}
}

#endif // IKCHECKCAPABILITY_H
