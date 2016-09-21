#ifndef SLIDING_CAPABILITY_H_
#define SLIDING_CAPABILITY_H_

#include <dual_manipulation_ik_control_capabilities/generic_planning_capability.h>
#include <atomic>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <bezier_curve/bezier_curve.h>
#include <ik_check_capability/ik_check_capability.h>

namespace dual_manipulation
{
namespace ik_control
{

class SlidingCapability : public GenericPlanningCapability
{
public:
    SlidingCapability(shared_ik_memory& sikm_, const ros::NodeHandle& node_ = ros::NodeHandle());
    virtual ~SlidingCapability();
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
//     void add_target(const dual_manipulation_shared::ik_service::Request& req);
    
private:
    shared_ik_memory& sikm;
    const ik_control_capability capabilities_;
    std::unique_ptr<ikCheckCapability> ik_check_;
    
    BezierCurve planner_bezier_curve;
    
//     std::mutex map_mutex_; // targets_
//     std::mutex robotState_mutex_;
//     
//     // keep an history of the required targets
//     std::map<std::string,dual_manipulation::ik_control::ik_target> targets_;
    
    // MoveIt! variables
    moveit::core::RobotModelPtr robot_model_;
//     moveit::core::RobotStatePtr target_rs_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    std::string robot_description;
//     planning_pipeline::PlanningPipelinePtr pipeline_;
//     planning_interface::MotionPlanRequest MotionPlanReq_;
    
    // ros variables
    ros::NodeHandle node;
    
    // planner parameters
//     std::string planner_id_;
//     double planning_time_;
//     std::string backup_planner_id_;
//     double backup_planning_time_;
//     int max_planning_attempts_;
//     int backup_max_planning_attempts_;
//     double goal_position_tolerance_;
//     double goal_orientation_tolerance_;
//     double goal_joint_tolerance_;
//     std::vector<double> ws_bounds_;
    
    // interface and results variables
    std::atomic_bool busy;
    dual_manipulation_shared::ik_response response_;
    
private:
    /**
     * @brief update a motionPlan request with a new target, considering the type of plan that will follow
     */
//     bool build_motionPlan_request(moveit_msgs::MotionPlanRequest& req, const std::map< std::string, dual_manipulation::ik_control::ik_target >& targets, ik_control_capabilities plan_type);
    
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
    void planSliding(const dual_manipulation_shared::ik_serviceRequest& req);
    void compute_orientation_from_vector(const Eigen::Vector3d& x_new, Eigen::Quaterniond& res);
    
    /**
     * @brief set the target robot state of the end-effector @p ee_name to the target specified in the SRDF with name @p named_target
     * 
     * @param ee_name the end-effector we want to set a target for
     * @param named_target the target name as specified in the SRDF
     * 
     * @return true on success
     */
//     bool set_target(std::string ee_name, std::string named_target);
};

}
}

#endif // SLIDING_CAPABILITY_H_
