#include "random_planning_capability.h"

using namespace dual_manipulation::ik_control;

randomPlanningCapability::randomPlanningCapability(shared_ik_memory& sikm_) : sikm(sikm_)
{
    reset();
}

randomPlanningCapability::~randomPlanningCapability()
{

}

bool randomPlanningCapability::canPerformCapability(const ik_control_capabilities& ik_capability)
{
    if ((ik_capability == ik_control_capabilities::PLAN) || (ik_capability == ik_control_capabilities::PLAN_BEST_EFFORT) || (ik_capability == ik_control_capabilities::PLAN_NO_COLLISION) || (ik_capability == ik_control_capabilities::PLAN_CLOSE_BEST_EFFORT))
        return true;
    
    return false;
}

void randomPlanningCapability::reset()
{
    // set default and parameter-dependent variable value
    
}

void randomPlanningCapability::performRequest(dual_manipulation_shared::ik_serviceRequest req)
{
    // using the request, perform the actual planning
}
