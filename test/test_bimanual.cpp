#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "dual_manipulation_shared/ik_service.h"
#include <atomic>

std::atomic_bool planning(false);

void check_callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Check : %s",str->data.c_str());
}

void check_callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Check %s",str->data.c_str());
}

void check_callback_bimanual(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Bimanual IK Check : %s",str->data.c_str());
}

void plan_callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Plan : %s",str->data.c_str());
    planning = false;
}

void plan_callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Plan : %s",str->data.c_str());
    planning = false;
}

void plan_callback_bimanual(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Bimanual IK Plan : %s",str->data.c_str());
    planning = false;
}

void exec_callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Exec : %s",str->data.c_str());
}

void exec_callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Exec : %s",str->data.c_str());
}

void exec_callback_bimanual(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Bimanual IK Exec : %s",str->data.c_str());
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> bimanual_ik_test "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "bimanual_ik_test");
    
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    ros::Subscriber check_lsub = n.subscribe("/ik_control/left_hand/check_done",1,check_callback_l);
    ros::Subscriber check_rsub = n.subscribe("/ik_control/right_hand/check_done",1,check_callback_r);
    ros::Subscriber check_bimanualsub = n.subscribe("/ik_control/both_hands/check_done",1,check_callback_bimanual);
    ros::Subscriber plan_lsub = n.subscribe("/ik_control/left_hand/planning_done",1,plan_callback_l);
    ros::Subscriber plan_rsub = n.subscribe("/ik_control/right_hand/planning_done",1,plan_callback_r);
    ros::Subscriber plan_bimanualsub = n.subscribe("/ik_control/both_hands/planning_done",1,plan_callback_bimanual);
    ros::Subscriber exec_lsub = n.subscribe("/ik_control/left_hand/action_done",1,exec_callback_l);
    ros::Subscriber exec_rsub = n.subscribe("/ik_control/right_hand/action_done",1,exec_callback_r);
    ros::Subscriber exec_bimanualsub = n.subscribe("/ik_control/both_hands/action_done",1,exec_callback_bimanual);
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
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
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
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
    sleep(1);
    
    srv.request.command = "plan";
    srv.request.ee_name = "both_hands";
    srv.request.ee_pose.push_back(ee_pose_r);

    planning = true;
    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
//     // this does not work, being the robot busy!
//     ROS_INFO("The following one should not work, being the robot busy!");
//     srv.request.ee_name = "left_hand";
// 
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
    
    int counter = 0;
    int max_wait = 20;
    while(planning && counter < max_wait)
    {
      ros::spinOnce();
      counter++;
      usleep(500000);
    }

    srv.request.command = "execute";
    srv.request.ee_name = "both_hands";
    
    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }

    for(int i=0; i<3; i++)
    {
      ros::spinOnce();
      usleep(500000);
    }

    return 0;
}