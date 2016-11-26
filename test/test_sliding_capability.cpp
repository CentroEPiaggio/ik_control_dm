#include <dual_manipulation_ik_control_capabilities/sliding_capability.h>
#include <dual_manipulation_shared/parsing_utils.h>

#define CLASS_NAMESPACE ""
#define DEBUG_STRING {std::cout << CLASS_NAMESPACE << __func__ << "@" << __LINE__ << std::endl;}

using namespace dual_manipulation::ik_control;

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> Test sliding_capability running"<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_sliding_capability");
    
    ros::NodeHandle node;
    ros::AsyncSpinner aspin(1);
    aspin.start();
    if(!ros::service::waitForService("/get_planning_scene"))
        return -1;
    
    if( ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Warn) & ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn) )
        ros::console::notifyLoggerLevelsChanged();

    XmlRpc::XmlRpcValue ik_control_params;
    bool params_ok = node.getParam("ik_control_parameters", ik_control_params);
        
    std::string joint_states_, robot_description_;
    
    joint_states_ = "/joint_states";
    robot_description_ = "/robot_description";

    if(!parseSingleParameter(ik_control_params,joint_states_,"joint_states"))
        ik_control_params["joint_states"] = joint_states_;
    if(!parseSingleParameter(ik_control_params,robot_description_,"robot_description"))
        ik_control_params["robot_description"] = robot_description_;
    
    std::shared_ptr<XmlRpc::XmlRpcValue> ik_control_params_ptr = std::make_shared<XmlRpc::XmlRpcValue>(ik_control_params);
    shared_ik_memory sikm(ik_control_params_ptr, node);
    
    sleep(1.0);
    SlidingCapability sliding_capability(sikm, node);
    
    dual_manipulation_shared::ik_serviceRequest plan_req;
    plan_req.ee_name = "right_hand";
    
    std::string group_name;
    if (!sikm.groupManager->getGroupInSRDF(plan_req.ee_name, group_name))
    {
        return -1;
    }
    sikm.resetPlanningRobotState(group_name);
    
    
    
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = -0.8;
    goal_pose.position.y = 0.1;
    goal_pose.position.z = 0.07;
    goal_pose.orientation.w = 1.0;
    plan_req.ee_pose.push_back(goal_pose);
    sliding_capability.add_target(plan_req);
    sliding_capability.performRequest(plan_req);
    moveit::planning_interface::MoveGroup::Plan plan;
    sikm.swapTrajectory(plan_req.ee_name,plan.trajectory_);
    char c;
    std::cout << "Press any key to send the trajectory to vito" << std::endl;
    std::cin >> c ;
    
    plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
    plan.trajectory_.joint_trajectory.points.erase(plan.trajectory_.joint_trajectory.points.begin());
    
    moveit::planning_interface::MoveItErrorCode res = sikm.robotController->asyncExecute(plan);
    if(moveit::planning_interface::MoveItErrorCode::SUCCESS != res.val)
    {
        ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : unable to send trajectory to the controller, MoveItErrorCode is " << res.val);
    }
    else
        sikm.robotController->waitForExecution(plan_req.ee_name, plan.trajectory_);
    ros::spin();

    return 0;
}