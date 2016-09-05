#ifndef ABSTRACT_CAPABILITY_H_
#define ABSTRACT_CAPABILITY_H_

#include <mutex>

#include <dual_manipulation_shared/ik_control_capabilities.h>
#include <dual_manipulation_shared/ik_serviceRequest.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <XmlRpcValue.h>

namespace dual_manipulation
{
namespace ik_control
{

/**
 * @brief A structure to share resources across implemented capabilities: plan, move, grasp, ...
 */
class shared_ik_memory
{
public:
    std::mutex m;
    planning_scene::PlanningSceneConstPtr plan_scene;
    XmlRpc::XmlRpcValue* ik_control_params;
};

/**
 * @brief An abstract class implementing a generic capability for use inside ik_control: plan, move, grasp, ...
 */
class abstractCapability
{
public:
    abstractCapability(){}
    virtual ~abstractCapability(){}
    virtual bool isComplete()=0;
    virtual void performRequest(dual_manipulation_shared::ik_serviceRequest req)=0;
    virtual bool canRun()=0;
    /// could be associated with a type coming from ik_control_capabilities.h
    virtual bool canPerformCapability(const ik_control_capabilities& ik_capability)=0;
    virtual void reset(){}
};

}
}

#endif // ABSTRACT_CAPABILITY_H_
