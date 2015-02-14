#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/scene_object_service.h"

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

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
    ros::ServiceClient client_obj = n.serviceClient<dual_manipulation_shared::scene_object_service>("scene_object_ros_service");
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
    dual_manipulation_shared::scene_object_service srv_obj;
    
    // create an object for grasping
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "";
    // the frame where the object position is considered (only when inserted, then it's *probably* fixed in the environment)
    attached_object.object.header.frame_id = "world";
    attached_object.object.id = "cylinder";
    /* A default pose */
    geometry_msgs::Pose obj_pose;
    obj_pose.position.x = -0.5;
    obj_pose.position.y = 0.0;
    obj_pose.position.z = 0.1;
    obj_pose.orientation.w = 1.0;
    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.05;
    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(obj_pose);
    
    /* An attach operation requires an ADD */
    attached_object.object.operation = attached_object.object.ADD;
    
    /////////////////////// add an object in the scene ///////////////////////
    srv_obj.request.command = "add";
    srv_obj.request.attObject = attached_object;
    
    if (client_obj.call(srv_obj))
    {
	ROS_INFO("IK_control:test_grasping : %s object %s request accepted: %d", srv_obj.request.command.c_str(),srv_obj.request.attObject.object.id.c_str(), (int)srv_obj.response.ack);
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::scene_object_service: %s %s",srv_obj.request.command.c_str(),srv_obj.request.attObject.object.id.c_str());
    }

    /////////////////////// plan to move the arm close to the object ///////////////////////
    srv.request.command = "plan";
    srv.request.ee_name = "left_hand";
    // compute hand pose
    geometry_msgs::Pose hand_pose = obj_pose;
    hand_pose.position.x += 0.05;
    hand_pose.position.z += primitive.dimensions.at(0)/2.0 + 0.05;
    KDL::Rotation rot;
    rot.EulerZYX(M_PI,M_PI/2.0,0.0).GetQuaternion(hand_pose.orientation.x,hand_pose.orientation.y,hand_pose.orientation.z,hand_pose.orientation.w);
    srv.request.ee_pose.clear();
    srv.request.ee_pose.push_back(hand_pose);
    
    if (client.call(srv))
    {
	ROS_INFO("IK_control:test_grasping : %s object request accepted: %d", srv_obj.request.command.c_str(), (int)srv_obj.response.ack);
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
    sleep(1);
    
    /////////////////////// actually move the arm ///////////////////////
    srv.request.command = "execute";
    
    if (client.call(srv))
    {
	ROS_INFO("IK_control:test_grasping : %s object request accepted: %d", srv_obj.request.command.c_str(), (int)srv_obj.response.ack);
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
    sleep(5);
    
    /////////////////////// perform grasp ///////////////////////
    
    // compute hand poses (this should not give problems for collision)
    hand_pose.position.z += 0.05;
    srv.request.ee_pose.clear();
    srv.request.ee_pose.push_back(hand_pose);
    hand_pose.position.z -= 0.1;
    srv.request.ee_pose.push_back(hand_pose);
    
    // object to be grasped
    attached_object.link_name = "left_hand_palm_link";
    attached_object.object.header.frame_id = "left_hand_palm_link";
    attached_object.object.header.stamp = ros::Time::now();
    // object pose relative to the palm: post-grasp pose from DB!
    obj_pose.position.x = primitive.dimensions.at(1);
    obj_pose.position.z = 0.05;
    rot.EulerZYX(0.0,0.0,M_PI/2.0).GetQuaternion(obj_pose.orientation.x,obj_pose.orientation.y,obj_pose.orientation.z,obj_pose.orientation.w);
    attached_object.object.primitive_poses.clear();
    attached_object.object.primitive_poses.push_back(obj_pose);
    
    // hand joint trajectory
    trajectory_msgs::JointTrajectory grasp_traj;
    grasp_traj.header.stamp = ros::Time::now();
    grasp_traj.joint_names.push_back(srv.request.ee_name + "_synergy_joint");
    std::vector <double > q = {0.0,1.0,0.5};
    trajectory_msgs::JointTrajectoryPoint tmp_traj;
    tmp_traj.positions.reserve(1);
    for (int i=0; i<q.size(); ++i)
    {
      tmp_traj.positions.clear();
      tmp_traj.positions.push_back(q.at(i));
      grasp_traj.points.push_back(tmp_traj);
    }
    
    // complete request
    srv.request.command = "grasp";
    srv.request.attObject = attached_object;
    srv.request.grasp_trajectory = grasp_traj;
    
    if (client.call(srv))
    {
	ROS_INFO("IK_control:test_grasping : %s object request accepted: %d", srv_obj.request.command.c_str(), (int)srv_obj.response.ack);
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
    sleep(5);
    
    
    
//     srv.request.attObject = attached_object;
//     // a vector in the request cannot be empty
//     srv.request.ee_pose.push_back(geometry_msgs::Pose());
//  
//     srv.request.ee_name = "both_hands";
//     srv.request.command = "grasp";
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
//     
//     srv.request.ee_name = "left_hand";
//     srv.request.command = "grasp";
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
//     
//     srv.request.ee_name = "right_hand";
//     srv.request.command = "grasp";
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
//     
//     srv.request.ee_name = "left_hand";
//     srv.request.command = "ungrasp";
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
//     
//     srv.request.ee_name = "right_hand";
//     srv.request.command = "ungrasp";
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
    
    ros::spinOnce();

    return 0;
}