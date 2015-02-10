#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/scene_object_service.h"

#include <visualization_msgs/Marker.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> ik_control : test_kin_grasping "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_kin_grasping");
    
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    ros::ServiceClient client_obj = n.serviceClient<dual_manipulation_shared::scene_object_service>("scene_object_ros_service");

    dual_manipulation_shared::ik_service srv;
    dual_manipulation_shared::scene_object_service srv_obj;
    
    // create an object to put in the scene
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
    
    srv_obj.request.command = "add";
    srv_obj.request.attObject = attached_object;
    
    if (client_obj.call(srv_obj))
    {
	ROS_INFO("IK_control:test_grasping : %s object request accepted: %d", srv_obj.request.command.c_str(), (int)srv_obj.response.ack);
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::scene_object_service");
    }
    
    sleep(5);
    
    srv_obj.request.command = "attach";
    srv_obj.request.attObject.object.header.frame_id = "left_hand_palm_link";
    
    if (client_obj.call(srv_obj))
    {
	ROS_INFO("IK_control:test_grasping : %s object request accepted: %d", srv_obj.request.command.c_str(), (int)srv_obj.response.ack);
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::scene_object_service");
    }
    
    sleep(5);
    
    srv_obj.request.command = "detach";
    srv_obj.request.attObject.object.header.frame_id = "world";
    
    if (client_obj.call(srv_obj))
    {
	ROS_INFO("IK_control:test_grasping : %s object request accepted: %d", srv_obj.request.command.c_str(), (int)srv_obj.response.ack);
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::scene_object_service");
    }
    
    sleep(5);
    
    srv_obj.request.command = "remove";
    srv_obj.request.object_id = "wrong_object";
    
    if (client_obj.call(srv_obj))
    {
	ROS_INFO("IK_control:test_grasping : %s object request accepted: %d", srv_obj.request.command.c_str(), (int)srv_obj.response.ack);
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::scene_object_service");
    }
    
    sleep(1);
    
    srv_obj.request.command = "remove";
    srv_obj.request.object_id = attached_object.object.id;
    
    if (client_obj.call(srv_obj))
    {
	ROS_INFO("IK_control:test_grasping : %s object request accepted: %d", srv_obj.request.command.c_str(), (int)srv_obj.response.ack);
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::scene_object_service");
    }
    
//     geometry_msgs::Pose ee_pose_r, ee_pose_l;
//     
//     // obtained via direct kinematics: left_hand_palm_link
//     // pos [x y z]: 8.55678e-08 1.1292 1.06425
//     // orient [x y z w]: -0.433013 0.25 0.433013 0.75
//     ee_pose_r.position.x = 0.2;
//     ee_pose_r.position.y = -0.2+1.1292;
//     ee_pose_r.position.z = -0.2+1.06425;
//     ee_pose_r.orientation.x = 0.0; //-0.433013;
//     ee_pose_r.orientation.y = 0.0; //0.25;
//     ee_pose_r.orientation.z = 0.0; //0.433013;
//     ee_pose_r.orientation.w = 1.0; //0.75;
// 
//     srv.request.command = "ik_check";
//     srv.request.ee_name = "right_hand";
//     srv.request.time = 2;
//     srv.request.ee_pose.push_back(ee_pose_r);
//     
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
//     }
// 
//     sleep(1);
//     
//     ee_pose_l = ee_pose_r;
//     ee_pose_l.position.x = -0.2;
//     ee_pose_l.position.y = 0.2-1.1292;
//     ee_pose_l.position.z = -0.2+1.06425;
// 
//     srv.request.command = "ik_check";
//     srv.request.ee_name = "left_hand";
//     srv.request.time = 2;
//     srv.request.ee_pose.clear();
//     srv.request.ee_pose.push_back(ee_pose_l);
//     
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
//     }
//     
//     sleep(1);
//     
//     srv.request.command = "plan";
//     srv.request.ee_name = "both_hands";
//     srv.request.ee_pose.push_back(ee_pose_r);
// 
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
//     }
//     
//     // this should not work, being the robot busy!
//     srv.request.ee_name = "left_hand";
// 
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
//     }
//     
//     sleep(1);
// 
//     srv.request.command = "execute";
//     
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
//     }

    ros::spin();

    return 0;
}
