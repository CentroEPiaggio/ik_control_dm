#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "dual_manipulation_shared/ik_service.h"

void callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Control : %s",str->data.c_str());
}

void callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Control : %s",str->data.c_str());
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> ik_test "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "ik_test");
    
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    ros::Subscriber lsub = n.subscribe("/ik_control/left_hand/action_done",1,callback_l);
    ros::Subscriber rsub = n.subscribe("/ik_control/right_hand/action_done",1,callback_r);
    dual_manipulation_shared::ik_service srv;
    
//     srv.request.ee_name = "left_hand";
//     srv.request.time = 2;
//     srv.request.ee_pose.position.x = 0.2;
//     srv.request.ee_pose.position.y = -0.4;
//     srv.request.ee_pose.position.z = 0.6;
//     srv.request.ee_pose.orientation.w = 0.5;
//     srv.request.ee_pose.orientation.x = 0.5;
//     srv.request.ee_pose.orientation.y = 0;
//     srv.request.ee_pose.orientation.z = 0;
//     
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
// 	return 1;
//     }
    
    srv.request.ee_name = "right_hand";
    srv.request.time = 2;
    srv.request.ee_pose.position.x = 1.0;
    srv.request.ee_pose.position.y = -0.5;
    srv.request.ee_pose.position.z = 1.0;
    srv.request.ee_pose.orientation.w = 0.5;
    srv.request.ee_pose.orientation.x = 0;
    srv.request.ee_pose.orientation.y = 0;
    srv.request.ee_pose.orientation.z = 0.5;
    
    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
	return 1;
    }

    ros::spin();

    return 0;
}