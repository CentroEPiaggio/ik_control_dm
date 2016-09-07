#ifndef RANDOM_PLANNING_CAPABILITY_H_
#define RANDOM_PLANNING_CAPABILITY_H_

#include "abstract_capability.h"
#include "ik_control.h" // needed for ik_target...
#include <atomic>

namespace dual_manipulation
{
namespace ik_control
{

class randomPlanningCapability : public abstractCapability
{
public:
    randomPlanningCapability(shared_ik_memory& sikm_);
    virtual ~randomPlanningCapability();
    virtual bool isComplete();
    virtual void performRequest(dual_manipulation_shared::ik_serviceRequest req);
    virtual bool getResults(dual_manipulation_shared::ik_response& res);
    virtual bool canRun();
    virtual bool canPerformCapability(const ik_control_capabilities& ik_capability) const;
    virtual void reset();
    
    /**
     * @brief add a target to the internal targets list
     * 
     * @param req the same req from the @e ik_service
     */
    void add_target(const dual_manipulation_shared::ik_service::Request& req);
    
private:
    shared_ik_memory& sikm;
    ik_control_capability capabilities_;
    
    std::mutex map_mutex_; 
    std::map<std::string,std::string> group_map_;
    std::vector<std::string> chain_names_list_;
    std::vector<std::string> tree_names_list_;
    std::map<std::string,std::vector<std::string>> tree_composition_;
    
    // keep an history of the required targets
    std::map<std::string,dual_manipulation::ik_control::ik_target> targets_;
    
    // MoveIt! variables
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr target_rs_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    planning_pipeline::PlanningPipelinePtr pipeline_;
    planning_interface::MotionPlanRequest MotionPlanReq_;
    
    // ros variables
    ros::NodeHandle node;
    
    // planner parameters
    std::string planner_id_;
    double planning_time_;
    std::string backup_planner_id_;
    double backup_planning_time_;
    int max_planning_attempts_;
    int backup_max_planning_attempts_;
    double goal_position_tolerance_;
    double goal_orientation_tolerance_;
    double goal_joint_tolerance_;
    std::vector<double> ws_bounds_;
    
    // interface and results variables
    std::atomic_bool busy;
    dual_manipulation_shared::ik_response plan_response;
    
private:
    /**
     * @brief update a motionPlan request with a new target, considering the type of plan that will follow
     */
    bool build_motionPlan_request(moveit_msgs::MotionPlanRequest& req, const std::map< std::string, dual_manipulation::ik_control::ik_target >& targets, ik_control_capabilities plan_type);
    
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
     * @brief set the target robot state of the end-effector @p ee_name to the target specified in the SRDF with name @p named_target
     * 
     * @param ee_name the end-effector we want to set a target for
     * @param named_target the target name as specified in the SRDF
     * 
     * @return true on success
     */
    bool set_target(std::string ee_name, std::string named_target);
};

}
}

#endif // RANDOM_PLANNING_CAPABILITY_H_
