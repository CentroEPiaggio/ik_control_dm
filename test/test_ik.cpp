#include "ros/ros.h"
#include <iostream>
#include "dual_manipulation_shared/ik_service.h"

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> ik_test "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "ik_test");
    
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    dual_manipulation_shared::ik_service srv;
    
    srv.request.ee_name = "left_hand";
    srv.request.time = 1;
    srv.request.ee_pose.position.x = 1;
    srv.request.ee_pose.position.y = 2;
    srv.request.ee_pose.position.z = 3;
    srv.request.ee_pose.orientation.w = 1;
    srv.request.ee_pose.orientation.x = 0;
    srv.request.ee_pose.orientation.y = 0;
    srv.request.ee_pose.orientation.z = 0;
    
    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
	return 1;
    }
    
    sleep(2);
    
    srv.request.ee_name = "right_hand";
    srv.request.time = 77;
    srv.request.ee_pose.position.x = 7;
    srv.request.ee_pose.position.y = 7;
    srv.request.ee_pose.position.z = 7;
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