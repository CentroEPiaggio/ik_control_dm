#include <dual_manipulation_ik_control_capabilities/sliding_capability.h>
#include <dual_manipulation_shared/parsing_utils.h>
#include <atomic>

#define CLASS_NAMESPACE ""
#define DEBUG_STRING {std::cout << CLASS_NAMESPACE << __func__ << "@" << __LINE__ << std::endl;}

using namespace dual_manipulation::ik_control;
std::atomic_int max_seq_no_received;

void check_seq_no(const dual_manipulation_shared::ik_responseConstPtr& res)
{
    if (max_seq_no_received.load() < res->seq)
        max_seq_no_received.store(res->seq);
}

int main(int argc, char **argv)
{
    max_seq_no_received.store(0);
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> Test sliding_capability running"<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_sliding_capability");
    
    ros::NodeHandle node;
    ros::ServiceClient sclient = node.serviceClient<dual_manipulation_shared::ik_service>("/ik_ros_service");
    ros::Subscriber sub1 = node.subscribe<dual_manipulation_shared::ik_response>("ik_control/planning_done", 1, &check_seq_no);
    ros::Subscriber sub2 = node.subscribe<dual_manipulation_shared::ik_response>("ik_control/action_done", 1, &check_seq_no);
    
    ros::AsyncSpinner aspin(1);
    aspin.start();
    if(!ros::service::waitForService("/ik_ros_service"))
        return -1;
    
    dual_manipulation_shared::ik_serviceRequest plan_req;
    plan_req.ee_name = "right_hand";
    
    ik_control_capability capability;
    
    geometry_msgs::Pose goal_pose;
    dual_manipulation_shared::ik_serviceResponse plan_res;
    
    // -> Set initial target
    goal_pose.position.x = -0.8;
    goal_pose.position.y = 0.4;
    goal_pose.position.z = 0.27;
    goal_pose.orientation.y = 1.0;
    plan_req.ee_pose.push_back(goal_pose);
    plan_req.command = capability.name.at(ik_control_capabilities::SET_TARGET);
    plan_req.seq = 1;
    if(!sclient.call(plan_req, plan_res))
    {
        DEBUG_STRING;
        abort();
    }
    
    // -> Plan
    plan_req.command =  capability.name.at(ik_control_capabilities::PLAN);
    ++plan_req.seq;
    if(!sclient.call(plan_req, plan_res))
    {
        DEBUG_STRING;
        abort();
    }
    while(max_seq_no_received.load() < plan_req.seq)
    {
        usleep(5000);
    }   
    
    // -> execute
    plan_req.command =  capability.name.at(ik_control_capabilities::MOVE);
    ++plan_req.seq;
    if(!sclient.call(plan_req, plan_res))
    {
        DEBUG_STRING;
        abort();
    }
    while(max_seq_no_received.load() < plan_req.seq)
    {
        usleep(5000);
    }
    
    // -> Plan slide
    goal_pose.position.x = -0.8;
    goal_pose.position.y = 0.1;
    goal_pose.position.z = 0.07;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.w = 1.0;
    plan_req.ee_pose.clear();
    plan_req.ee_pose.push_back(goal_pose);
    plan_req.command = capability.name.at(ik_control_capabilities::SET_SLIDE_TARGET);
    ++plan_req.seq;
    if(!sclient.call(plan_req, plan_res))
    {
        DEBUG_STRING;
        abort();
    }
    
    plan_req.command =  capability.name.at(ik_control_capabilities::PLAN_SLIDE);
    ++plan_req.seq;
    if(!sclient.call(plan_req, plan_res))
    {
        DEBUG_STRING;
        abort();
    }
    while(max_seq_no_received.load() < plan_req.seq)
    {
        usleep(5000);
    }
    
    // -> Execute
    plan_req.command =  capability.name.at(ik_control_capabilities::MOVE);
    ++plan_req.seq;
    if(!sclient.call(plan_req, plan_res))
    {
        DEBUG_STRING;
        abort();
    }
    while(ros::ok() && max_seq_no_received.load() < plan_req.seq)
    {
        usleep(5000);
    }
    
    while(ros::ok())
    {
        ros::spinOnce();
        usleep(50000);
    }

    return 0;
}