#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "dual_manipulation_shared/ik_service.h"

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
}

void plan_callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Plan : %s",str->data.c_str());
}

void plan_callback_bimanual(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Bimanual IK Plan : %s",str->data.c_str());
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

void grasp_callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Grasp : %s",str->data.c_str());
}

void grasp_callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Grasp : %s",str->data.c_str());
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
    ros::Subscriber grasp_lsub = n.subscribe("/ik_control/left_hand/grasp_done",1,grasp_callback_l);
    ros::Subscriber grasp_rsub = n.subscribe("/ik_control/right_hand/grasp_done",1,grasp_callback_r);
    dual_manipulation_shared::ik_service srv;
    
    // create an object for grasping
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "left_hand_palm_link";
    /* The header must contain a valid TF frame*/
    // attached_object.object.header.frame_id = "left_hand_palm_link";
    attached_object.object.header.frame_id = "world";
    /* The id of the object */
    attached_object.object.id = "cylinder";
    /* A default pose */
    geometry_msgs::Pose pose;
    pose.position.x = -0.4;
    pose.position.y = 0.5;
    pose.position.z = 0.2;
    pose.orientation.w = 1.0;
    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.1;
    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);
    /* An attach operation requires an ADD */
    attached_object.object.operation = attached_object.object.ADD;
    
    srv.request.attObject = attached_object;

    // a vector in the request cannot be empty
    srv.request.ee_pose.push_back(geometry_msgs::Pose());
 
    srv.request.ee_name = "both_hands";
    srv.request.command = "grasp";
    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
    srv.request.ee_name = "left_hand";
    srv.request.command = "grasp";
    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
    srv.request.ee_name = "right_hand";
    srv.request.command = "grasp";
    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
    srv.request.ee_name = "left_hand";
    srv.request.command = "ungrasp";
    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
    srv.request.ee_name = "right_hand";
    srv.request.command = "ungrasp";
    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
    ros::spinOnce();

    return 0;
}