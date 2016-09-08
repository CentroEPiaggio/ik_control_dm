#include <dual_manipulation_ik_control/robot_state_manager.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <string>

#define CLASS_LOGNAME "RobotStateManager"
#define CLASS_NAMESPACE "RobotStateManager::"

RobotStateManager::RobotStateManager(const moveit::core::RobotModelConstPtr& robot_model, const std::string& joint_states)
{
    const boost::shared_ptr<tf::Transformer> tf(new tf::Transformer());
    current_state_monitor_.reset(new planning_scene_monitor::CurrentStateMonitor(robot_model,tf));
    current_state_monitor_->startStateMonitor(joint_states);
}

bool RobotStateManager::reset_robot_state(const moveit::core::RobotStatePtr& rs, const std::string& group, std::mutex& rs_mutex) const
{
    if (!updateCurrentState(group))
        return false;
    
    std::unique_lock<std::mutex> lck1(rs_mutex,std::defer_lock);
    std::unique_lock<std::mutex> lck2(cs_mutex_,std::defer_lock);
    std::lock(lck1,lck2);
    
    // check that we are using the same robot
    assert(current_state_->getRobotModel()->getName() == rs->getRobotModel()->getName());
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : resetting " << rs->getRobotModel()->getName());
    
    for(int i=0; i<rs->getVariableCount(); i++)
        rs->setVariablePosition(i,current_state_->getVariablePosition(i));
    
    return true;
}

bool RobotStateManager::reset_robot_state(const moveit::core::RobotStatePtr& rs, const std::string& group, std::mutex& rs_mutex, const moveit_msgs::RobotTrajectory& traj) const
{
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : resetting " << rs->getRobotModel()->getName() << " with a trajectory for group " << group);
    
    // can't use the "same robot" check here, so making it more complex (and more time-consuming...)
    assert(rs->getJointModelGroup(group)->getVariableCount() == traj.joint_trajectory.joint_names.size());
    for(int i=0; i<rs->getJointModelGroup(group)->getVariableCount(); ++i)
        assert(rs->getJointModelGroup(group)->getVariableNames().at(i).compare( traj.joint_trajectory.joint_names.at(i) ) == 0);
    
    //NOTE: robot_traj, built on robot_model, contains the full robot; trajectory, instead, is only for the group joints
    robot_trajectory::RobotTrajectory robot_traj(rs->getRobotModel(),group);
    robot_traj.setRobotTrajectoryMsg(*rs,traj);

    for(int i=0; i<rs->getVariableCount(); i++)
        rs->setVariablePosition(i,robot_traj.getLastWayPoint().getVariablePosition(i));
    
    return true;
}

bool RobotStateManager::updateCurrentState(const std::string& group, double wait_seconds) const
{
    std::unique_lock<std::mutex> ul(cs_mutex_);
    
    if (!current_state_monitor_ || !current_state_monitor_->isActive())
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unable to get current robot state");
        return false;
    }
    
    if (!current_state_monitor_->waitForCurrentState(group, wait_seconds))
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Joint values for monitored state are requested but the full state is not known");
    
    current_state_ = current_state_monitor_->getCurrentState();
    return true;
}