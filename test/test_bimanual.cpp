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

void callback_bimanual(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Bimanual IK Control : %s",str->data.c_str());
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> bimanual_ik_test "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "bimanual_ik_test");
    
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    ros::Subscriber lsub = n.subscribe("/ik_control/left_hand/action_done",0,callback_l);
    ros::Subscriber rsub = n.subscribe("/ik_control/right_hand/action_done",0,callback_r);
    ros::Subscriber bimanualsub = n.subscribe("/ik_control/both_hands/action_done",0,callback_bimanual);
    dual_manipulation_shared::ik_service srv;
    
    geometry_msgs::Pose ee_pose_r, ee_pose_l;
    
    // obtained via direct kinematics: left_hand_palm_link
    // pos [x y z]: 8.55678e-08 1.1292 1.06425
    // orient [x y z w]: -0.433013 0.25 0.433013 0.75
    ee_pose_r.position.x = 0.2;
    ee_pose_r.position.y = -0.2+1.1292;
    ee_pose_r.position.z = -0.2+1.06425;
    ee_pose_r.orientation.x = 0.0; //-0.433013;
    ee_pose_r.orientation.y = 0.0; //0.25;
    ee_pose_r.orientation.z = 0.0; //0.433013;
    ee_pose_r.orientation.w = 1.0; //0.75;

    srv.request.command = "ik_check";
    srv.request.ee_name = "right_hand";
    srv.request.time = 2;
    srv.request.ee_pose.push_back(ee_pose_r);
    
    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
    }

    sleep(1);
    
    ee_pose_l = ee_pose_r;
    ee_pose_l.position.x = -0.2;
    ee_pose_l.position.y = 0.2-1.1292;
    ee_pose_l.position.z = -0.2+1.06425;

    srv.request.command = "ik_check";
    srv.request.ee_name = "left_hand";
    srv.request.time = 2;
    srv.request.ee_pose.clear();
    srv.request.ee_pose.push_back(ee_pose_l);
    
    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
    }
    
    sleep(1);
    
    srv.request.command = "plan";
    srv.request.ee_name = "both_hands";
    srv.request.ee_pose.push_back(ee_pose_r);

    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
    }
    
    // this should not work, being the robot busy!
    srv.request.ee_name = "left_hand";

    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
    }
    
    sleep(1);

    srv.request.command = "execute";
    
    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
    }

    ros::spin();

    return 0;
}