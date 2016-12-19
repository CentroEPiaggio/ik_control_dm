#include <dual_manipulation_ik_control/robot_controller_interface.h>
#include <dual_manipulation_shared/parsing_utils.h>
#include <ros/ros.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/move_group/capability_names.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#define CLASS_NAMESPACE "RobotControllerInterface::"
#define CLASS_LOGNAME "RobotControllerInterface"

using namespace dual_manipulation::ik_control;

RobotControllerInterface::RobotControllerInterface(XmlRpc::XmlRpcValue& params, const GroupStructureManager& groupManager_, const RobotStateManager& rsManager_, const ros::NodeHandle& node_) : node(node_), groupManager(groupManager_), rsManager(rsManager_), initialized(false)
{
    busy.store(true);
    
    parseParameters(params);
    setParameterDependentVariables();
    
    resetRobotModel(rsManager.get_robot_state_copy()->getRobotModel());
}

void RobotControllerInterface::resetRobotModel(moveit::core::RobotModelConstPtr robot_model)
{
    assert(robot_model);
    
    robot_model_ = robot_model;
    visual_rs_.reset(new moveit::core::RobotState(robot_model_));
    
    resetMoveGroup();
}

MPErrorCode RobotControllerInterface::asyncExecute(const MotionPlan& plan) const
{
    std::unique_lock<std::mutex> ul(execution_mutex_);
    
    moveit_msgs::ExecuteKnownTrajectory::Request req;
    moveit_msgs::ExecuteKnownTrajectory::Response res;
    req.trajectory = plan.trajectory_;
    req.wait_for_execution = false;
    if (execute_service_.call(req, res))
    {
        return moveit::planning_interface::MoveItErrorCode(res.error_code);
    }
    else
    {
        return moveit::planning_interface::MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
    }
}

void RobotControllerInterface::setParameterDependentVariables()
{
    trajectory_event_publisher_ = node.advertise<std_msgs::String>(trajectory_execution_manager::TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC, 1, false);
    joint_state_pub_ = node.advertise<sensor_msgs::JointState>("ik_control_joint_states",10);
    
    for(auto chain_name:groupManager.get_chains())
    {
        // JointTrajectory publishers
        if(hand_synergy_pub_topics_.count(chain_name))
            hand_synergy_pub_[chain_name] = node.advertise<trajectory_msgs::JointTrajectory>(hand_synergy_pub_topics_.at(chain_name),1,this);
    }
}

void RobotControllerInterface::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    bool mandatory_params = true;
    mandatory_params = mandatory_params & parseSingleParameter(params,joint_states_,"joint_states");
    assert(mandatory_params);
    
    parseSingleParameter(params,position_threshold,"position_threshold");
    parseSingleParameter(params,velocity_threshold,"velocity_threshold");
    parseSingleParameter(params,hand_position_threshold,"hand_position_threshold");
    parseSingleParameter(params,kinematics_only_,"kinematics_only");
    
    auto chain_names = groupManager.get_chains();
    parseSingleParameter(params,hand_synergy_pub_topics_,"hand_synergy_pub_topics",chain_names);
    parseSingleParameter(params,controller_map_,"controller_map",chain_names);
    parseSingleParameter(params,hand_actuated_joint_,"hand_actuated_joint",chain_names);
}

void RobotControllerInterface::resetMoveGroup()
{
    if(!ros::service::waitForService(move_group::EXECUTE_SERVICE_NAME))
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : trajectory execution service NOT found!");
        return;
    }
    execute_service_ = node.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(move_group::EXECUTE_SERVICE_NAME);
    
    // this makes the class usable
    if(!initialized)
    {
        initialized = true;
        busy.store(false);
    }
}

bool RobotControllerInterface::moveHand(const std::string& hand, const std::vector< double >& q, std::vector< double >& t, trajectory_msgs::JointTrajectory& grasp_traj) const
{
    // if an end-effector is not a hand
    if(!hand_actuated_joint_.count(hand))
        return false;
    
    grasp_traj.joint_names.push_back(hand_actuated_joint_.at(hand));
    
    if (t.size() != q.size())
    {
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : timing vector size non compatible with joint vector size, using a default timing of 1 second");
        t.clear();
        for (int i=0; i<q.size(); ++i)
            t.push_back(1.0/q.size()*i);
    }
    
    trajectory_msgs::JointTrajectoryPoint tmp_traj;
    tmp_traj.positions.reserve(1);
    
    for (int i=0; i<q.size(); ++i)
    {
        tmp_traj.positions.clear();
        tmp_traj.positions.push_back(q.at(i));
        tmp_traj.time_from_start = ros::Duration(t.at(i));
        
        grasp_traj.points.push_back(tmp_traj);
    }
    
    std::unique_lock<std::mutex> ul(hand_synergy_pub_mutex_);
    hand_synergy_pub_.at(hand).publish(grasp_traj);
    
    return true;
}

bool RobotControllerInterface::moveHand(const std::string& hand, const trajectory_msgs::JointTrajectory& grasp_traj) const
{
    // if an end-effector is not a hand
    if(!hand_actuated_joint_.count(hand))
        return false;
    
    std::unique_lock<std::mutex> ul(hand_synergy_pub_mutex_);
    hand_synergy_pub_.at(hand).publish(grasp_traj);
    
    return true;
}

bool RobotControllerInterface::waitForHandMoved(std::string& hand, double hand_target, const trajectory_msgs::JointTrajectory& traj) const
{
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : entered");
    
    // if an end-effector is not a hand
    if(!hand_actuated_joint_.count(hand))
        return true;
    
    if(kinematics_only_)
    {
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : kinematics_only execution - moving on after the trajectory has been shown");
        moveit_msgs::RobotTrajectory traj2;
        traj2.joint_trajectory = traj;
        publishTrajectoryPath(traj2);
        return true;
    }
    
    int counter = 0;
    int hand_index = 0;
    bool good_stop = false;
    sensor_msgs::JointStateConstPtr joint_states;
    ros::NodeHandle nh(node);
    
    joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_states_,nh,ros::Duration(3));
    for(auto joint:joint_states->name)
    {
        if(joint == hand_actuated_joint_.at(hand))
        {
            break;
        }
        hand_index++;
    }
    if(hand_index >= joint_states->name.size())
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : " << hand_actuated_joint_.at(hand) << " NOT found in /joint_states - returning");
        return false;
    }
    
    // wait for up to 10 more seconds
    while(counter<100)
    {
        //get joint states
        joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_states_,nh,ros::Duration(3));
        
        if(joint_states->name.at(hand_index) != hand_actuated_joint_.at(hand))
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : joints in joint_states changed order");
            return false;
        }
        
        if (std::norm(joint_states->position.at(hand_index) - hand_target) < hand_position_threshold)
        {
            good_stop = true;
            break;
        }
        usleep(100000);
        counter++;
    }
    
    if(good_stop)
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : exiting with good_stop OK");
    else
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : exiting with error");
    return good_stop;
}

bool RobotControllerInterface::waitForExecution(std::string ee_name, moveit_msgs::RobotTrajectory traj) const
{
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : entered");
    
    if (traj.joint_trajectory.points.size() == 0)
    {
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the trajectory to wait for was empty");
        return true;
    }
    
    if(kinematics_only_)
    {
        publishTrajectoryPath(traj);
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : kinematics_only execution - moving on after the trajectory has been shown");
        return true;
    }
    
    map_mutex_.lock();
    int has_ctrl = controller_map_.count(ee_name);
    std::string controller_name;
    if(has_ctrl != 0)
        controller_name = controller_map_.at(ee_name);
    map_mutex_.unlock();
    
    control_msgs::FollowJointTrajectoryActionResultConstPtr pt;
    ros::Duration timeout = traj.joint_trajectory.points.back().time_from_start;
    ros::NodeHandle nh(node);
    if(has_ctrl != 0)
    {
        // only do this if a controller exists - use a scaled timeout
        timeout = timeout*1.0;
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : waiting for at most " << timeout << " (trajectory total time)");
        pt = ros::topic::waitForMessage<control_msgs::FollowJointTrajectoryActionResult>(controller_name + "result",nh,timeout);
        if(pt)
            ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : received message - error_code=" << pt->result.error_code);
        else
            ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : timeout reached");
    }
    else
    {
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : waiting for at least " << timeout << " (trajectory total time)");
        timeout.sleep();
    }
    
    std::vector<std::string> joints = traj.joint_trajectory.joint_names;
    std::vector<double> q,Dq,goal_q;
    q.reserve(joints.size());
    Dq.reserve(joints.size());
    goal_q.reserve(joints.size());
    
    // (remember that the goal position is the last pose of the trajectory)
    for(auto q_i:traj.joint_trajectory.points.back().positions)
        goal_q.push_back(q_i);
    
    // goal size and joints size MUST be equal: check
    assert(goal_q.size() == joints.size());
    
    double vel,dist;
    bool good_stop = false;
    
    sensor_msgs::JointStateConstPtr joint_states;
    
    // wait until the robot is moving
    vel = 1.0 + velocity_threshold;
    while(vel > velocity_threshold)
    {
        q.clear();
        Dq.clear();
        vel = 0.0;
        dist = 0.0;
        
        // TODO: use one subscriber instead of waiting on single messages?
        joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_states_,nh,ros::Duration(3));
        bool joint_found;
        
        //get joint states
        for(auto joint:joints)
        {
            joint_found = false;
            for(int i=0; i<joint_states->name.size(); i++)
            {
                if(joint == joint_states->name.at(i))
                {
                    q.push_back(joint_states->position.at(i));
                    Dq.push_back(joint_states->velocity.at(i));
                    joint_found = true;
                    break;
                }
            }
            if(!joint_found)
            {
                ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : couldn't find requested joints!");
                return false;
            }
        }
        
        for(auto v:Dq)
            vel += std::norm(v);
        
        //if velocity < eps1
        if (vel<velocity_threshold)
        {
            for(int i=0; i<q.size(); i++)
                dist += std::norm(q.at(i) - goal_q.at(i));
            
            //if norm(fk(position) - goal) < eps2
            if (dist<position_threshold)
            {
                good_stop = true;
                break;
            }
            else
            {
                ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : vel=" << vel << " (< " << velocity_threshold << ") but dist=" << dist << " (>= " << position_threshold << ")");
                break;
            }
        }
        usleep(100000);
    }
    
    if(good_stop)
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : exiting with good_stop OK");
    else
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : exiting with error");
    return good_stop;
}

bool RobotControllerInterface::publishTrajectoryPath(const moveit_msgs::RobotTrajectory& trajectory_msg) const
{
    std::string group_name;
    bool exists = groupManager.getGroupInSRDF("full_robot",group_name);
    assert(exists);
    robot_trajectory::RobotTrajectory trajectory(robot_model_,group_name);
    std::mutex tmp_mutex;
    rsManager.reset_robot_state(visual_rs_,group_name,tmp_mutex);
    trajectory.setRobotTrajectoryMsg(*(visual_rs_),trajectory_msg);
    ros::Duration dTs(0.1);
    ros::Duration time(0);
    
    sensor_msgs::JointState js_msg;
    js_msg.name = trajectory_msg.joint_trajectory.joint_names;
    js_msg.header = trajectory_msg.joint_trajectory.header;
    ros::Duration total_time = trajectory_msg.joint_trajectory.points.back().time_from_start;
    
    ros::Rate rate(1.0/dTs.toSec());
    while(time < total_time)
    {
        time += dTs;
        if(time > total_time)
            time = total_time;
        
        if(!trajectory.getStateAtDurationFromStart(time.toSec(),visual_rs_))
        {
            ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to get interpolated state, trajectory.getStateAtDurationFromStart(...) failed");
            return false;
        }
        
        js_msg.header.stamp = ros::Time::now();
        js_msg.position.clear();
        js_msg.velocity.clear();
        for(int i=0; i<js_msg.name.size(); i++)
        {
            js_msg.position.push_back(visual_rs_->getVariablePosition(js_msg.name.at(i)));
            js_msg.velocity.push_back(visual_rs_->getVariableVelocity(js_msg.name.at(i)));
        }
        joint_state_pub_.publish(js_msg);
        ros::spinOnce();
        
        rate.sleep();
    }
    
    return true;
}
