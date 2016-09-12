#include <dual_manipulation_ik_control_capabilities/trajectory_execution_capability.h>
#include <dual_manipulation_shared/parsing_utils.h>

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
    
    if(!sikm.setPendingTrajectoryExecution())
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unable to set a pending trajectory execution - returning...");
        return;
    }
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Executing plan for " << req.ee_name);
    
    moveit::planning_interface::MoveItErrorCode error_code;
    moveit::planning_interface::MoveGroup::Plan movePlan;
    moveit_msgs::RobotTrajectory& traj(movePlan.trajectory_);
    
    // to be sure that no other execution is tried, swap with an empty trajectory
    sikm.swapTrajectory(req.ee_name,traj);
    
    // old execution method: does not allow for two trajectories at the same time
    if(!kinematics_only_)
        error_code = sikm.robotController->asyncExecute(movePlan);
    
    ros::Duration dt(0.0);
    if(!movePlan.trajectory_.joint_trajectory.points.empty())
        dt = movePlan.trajectory_.joint_trajectory.points.back().time_from_start;
    
    if(!sikm.setNextTrajectoryRelativeEndTime(dt))
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unable to set trajectory end-time: maybe someone executed it already?");
    
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
        sikm.resetPlanningRobotState(group_name);
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
