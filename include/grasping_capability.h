#ifndef GRASPING_CAPABILITY_H_
#define GRASPING_CAPABILITY_H_

#include "abstract_capability.h"
#include "ik_check_capability/ik_check_capability.h"
#include <atomic>
#include <moveit/robot_model_loader/robot_model_loader.h>

namespace dual_manipulation
{
namespace ik_control
{

class GraspingCapability : public abstractCapability
{
public:
    GraspingCapability(shared_ik_memory& sikm_, const ros::NodeHandle& node_ = ros::NodeHandle());
    virtual ~GraspingCapability();
    virtual bool isComplete();
    virtual void performRequest(dual_manipulation_shared::ik_serviceRequest req);
    virtual bool getResults(dual_manipulation_shared::ik_response& res);
    virtual bool canRun();
    virtual bool canPerformCapability(const ik_control_capabilities& ik_capability) const;
    virtual void reset();
    
private:
    shared_ik_memory& sikm;
    bool kinematics_only_;
    std::unique_ptr<ikCheckCapability> ik_check_;
    std::mutex ikCheck_mutex_;
    double hand_max_velocity;   // maximum hand velocity : avg is 2.0, closes completely [0.0->1.0] in half a second
    std::map<std::string,std::vector<std::string>> allowed_collision_prefixes_;
    std::map<std::string,std::vector<std::string>> allowed_collisions_;
    std::map<std::string,std::string> hand_actuated_joint_;
    std::mutex map_mutex_; // hand_actuated_joint_
    const ik_control_capability capabilities_;
    
    // ros variables
    ros::NodeHandle node;
    // TODO: remove this in favor of a shared planning_scene_monitor
    ros::ServiceClient scene_client_;
    
    // interface and results variables
    std::atomic_bool busy;
    dual_manipulation_shared::ik_response response_;
    
    // MoveIt! variables
    moveit::core::RobotModelPtr robot_model_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    
private:
    /**
     * @brief Utility function to parse parameters
     * 
     * @param params params got from the parameter server
     */
    void parseParameters(XmlRpc::XmlRpcValue& params);
    
    /**
     * @brief Utility function to set class variables which depend on parameters
     */
    void setParameterDependentVariables();
    
    /**
     * @brief handler function for grasping an object
     * 
     * @param req the same req from the @e ik_service
     */
    void grasp(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief handler function for ungrasping an object
     * 
     * @param req the same req from the @e ik_service
     */
    void ungrasp(dual_manipulation_shared::ik_service::Request req);
};

}
}

#endif // GRASPING_CAPABILITY_H_
