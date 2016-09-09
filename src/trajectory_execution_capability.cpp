#include "trajectory_execution_capability.h"
#include <dual_manipulation_shared/parsing_utils.h>
//TODO: remove this when robotController will no longer depend on ik_shared_memory
#include <dual_manipulation_ik_control/robot_controller_interface.h>

#define CLASS_NAMESPACE "TrajectoryExecutionCapability::"
#define CLASS_LOGNAME "TrajectoryExecutionCapability"

using namespace dual_manipulation::ik_control;

TrajectoryExecutionCapability::TrajectoryExecutionCapability(dual_manipulation::ik_control::shared_ik_memory& sikm_, const ros::NodeHandle& node_) : sikm(sikm_), node(node_)
{
    reset();
}

TrajectoryExecutionCapability::~TrajectoryExecutionCapability() { }

bool TrajectoryExecutionCapability::canPerformCapability(const ik_control_capabilities& ik_capability) const
{
    if (ik_capability == ik_control_capabilities::MOVE)
        return true;
    
    return false;
}

void TrajectoryExecutionCapability::parseParameters(XmlRpc::XmlRpcValue& params)
{
    parseSingleParameter(params,kinematics_only_,"kinematics_only");
}

void TrajectoryExecutionCapability::setParameterDependentVariables()
{
    
}

void TrajectoryExecutionCapability::reset()
{
    // set default and parameter-dependent variable value
    {
        std::unique_lock<std::mutex> ul(sikm.m);
        parseParameters(*(sikm.ik_control_params));
    }
    
    setParameterDependentVariables();
    
    busy.store(false);
}

void TrajectoryExecutionCapability::performRequest(dual_manipulation_shared::ik_serviceRequest req)
{
    // tell the interface I'm busy
    bool I_am_busy = busy.exchange(true);
    if(I_am_busy)
        return;
    
    sikm.end_time_mutex_.lock();
    sikm.movement_end_time_ = ros::Time(0);
    sikm.end_time_mutex_.unlock();
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Executing plan for " << req.ee_name);
    
    moveit::planning_interface::MoveItErrorCode error_code;
    moveit::planning_interface::MoveGroup::Plan movePlan;
    
    sikm.movePlans_mutex_.lock();
    //NOTE: to be sure that no other execution is tried using this movePlan, use swap
    std::swap(movePlan,sikm.movePlans_.at(req.ee_name));
    sikm.movePlans_mutex_.unlock();
    
    // old execution method: does not allow for two trajectories at the same time
    if(!kinematics_only_)
        error_code = sikm.robotController->asyncExecute(movePlan);
    
    if(!movePlan.trajectory_.joint_trajectory.points.empty())
    {
        std::unique_lock<std::mutex> ul(sikm.end_time_mutex_);
        sikm.movement_end_time_ = ros::Time::now() + movePlan.trajectory_.joint_trajectory.points.back().time_from_start;
    }
    bool good_stop = sikm.robotController->waitForExecution(req.ee_name,movePlan.trajectory_);
    
    response_.seq=req.seq;
    response_.group_name = req.ee_name;
    
    if(good_stop)
    {
        response_.data = "done";
    }
    else
    {
        std::string group_name;
        sikm.groupManager->getGroupInSRDF(req.ee_name,group_name);
        sikm.robotStateManager->reset_robot_state(sikm.planning_init_rs_,group_name,sikm.robotState_mutex_);
        response_.data = "error";
    }
    
    busy.store(false);
}

bool TrajectoryExecutionCapability::isComplete()
{
    return !busy.load();
}

bool TrajectoryExecutionCapability::canRun()
{
    return !busy.load();
}

bool TrajectoryExecutionCapability::getResults(dual_manipulation_shared::ik_response& res)
{
    // fill response message
    if(busy.load())
        return false;
    
    res = response_;
    return true;
}
