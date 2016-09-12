#ifndef TRAJECTORY_EXECUTION_CAPABILITY_H_
#define TRAJECTORY_EXECUTION_CAPABILITY_H_

#include <dual_manipulation_ik_control_capabilities/abstract_capability.h>
#include <atomic>

namespace dual_manipulation
{
namespace ik_control
{

class TrajectoryExecutionCapability : public abstractCapability
{
public:
    TrajectoryExecutionCapability(shared_ik_memory& sikm_, const ros::NodeHandle& node_ = ros::NodeHandle());
    virtual ~TrajectoryExecutionCapability();
    virtual bool isComplete();
    virtual void performRequest(dual_manipulation_shared::ik_serviceRequest req);
    virtual bool getResults(dual_manipulation_shared::ik_response& res);
    virtual bool canRun();
    virtual bool canPerformCapability(const ik_control_capabilities& ik_capability) const;
    virtual void reset();
    
private:
    shared_ik_memory& sikm;
    bool kinematics_only_;
    
    // ros variables
    ros::NodeHandle node;
    
    // interface and results variables
    std::atomic_bool busy;
    dual_manipulation_shared::ik_response response_;
    
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
};

}
}

#endif // TRAJECTORY_EXECUTION_CAPABILITY_H_
