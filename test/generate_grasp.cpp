#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/scene_object_service.h"

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <ros/package.h>
#include <fstream>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>

#define hand_position_threshold 1.0/200.0

ros::NodeHandle* nodePtr;

void publish_marker_utility(ros::Publisher& vis_pub, geometry_msgs::Pose& pose, double scale=0.05)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
//   marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = scale/2.0;
  marker.scale.y = scale;
  marker.scale.z = scale*2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  vis_pub.publish( marker );
  ros::spinOnce();
}

int action_count,gotobject_count;

void check_callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Check : %s",str->data.c_str());
    action_count--;
}

void check_callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Check %s",str->data.c_str());
    action_count--;
}

void check_callback_bimanual(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Bimanual IK Check : %s",str->data.c_str());
    action_count--;
}

void plan_callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Plan : %s",str->data.c_str());
    action_count--;
}

void plan_callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Plan : %s",str->data.c_str());
    action_count--;
}

void plan_callback_bimanual(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Bimanual IK Plan : %s",str->data.c_str());
    action_count--;
}

void exec_callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Exec : %s",str->data.c_str());
    action_count--;
}

void exec_callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Exec : %s",str->data.c_str());
    action_count--;
}

void exec_callback_bimanual(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Bimanual IK Exec : %s",str->data.c_str());
    action_count--;
}

void grasp_callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Grasp : %s",str->data.c_str());
    action_count--;
}

void grasp_callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Grasp : %s",str->data.c_str());
    action_count--;
}

gazebo_msgs::ModelStates gazebo_mdl;

void object_pose_callback(const gazebo_msgs::ModelStates::ConstPtr& gazebo_mdl_states)
{
    if (--gotobject_count >= 0)
    {
	ROS_INFO("Cylinder got from Gazebo");
	gazebo_mdl = *gazebo_mdl_states;
    }
}

std::map<std::string,std::string> hand_actuated_joint;

bool waitForHandMoved(std::string& hand, double hand_target)
{
  ROS_INFO_STREAM("waitForHandMoved : entered");

  int counter = 0;
  int hand_index = 0;
  double vel,dist;
  bool good_stop = false;
  sensor_msgs::JointStateConstPtr joint_states;
  
  joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",*nodePtr,ros::Duration(3));
  for(auto joint:joint_states->name)
  {
    if(joint == hand_actuated_joint.at(hand))
    {
      break;
    }
    hand_index++;
  }
  if(hand_index >= joint_states->name.size())
  {
    ROS_ERROR_STREAM("waitForHandMoved : " << hand_actuated_joint.at(hand) << " NOT found in /joint_states - returning");
    return false;
  }

  while(counter<20)
  {
    //get joint states
    joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",*nodePtr,ros::Duration(3));
    
    if(joint_states->name.at(hand_index) != hand_actuated_joint.at(hand))
    {
      ROS_ERROR("waitForHandMoved : joints in joint_states changed order");
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
    ROS_INFO("ikControl::waitForHandMoved : exiting with good_stop OK");
  else
    ROS_WARN("ikControl::waitForHandMoved : exiting with error");
  return good_stop;
}

std::map<std::string,ros::Publisher> hand_synergy_pub;

bool moveHand(std::string& hand, std::vector< double >& q, std::vector< double >& t)
{
  trajectory_msgs::JointTrajectory grasp_traj;
  
//   grasp_traj.header.stamp = ros::Time::now();
  grasp_traj.joint_names.push_back(hand + "_synergy_joint");
  
  if (t.size() != q.size())
  {
    ROS_WARN("IKControl::moveHand: timing vector size non compatible with joint vector size, using a default timing of 1 second");
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
  
  hand_synergy_pub.at(hand).publish(grasp_traj);
  
  ros::spinOnce();
  
  return true;
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> generate_grasp "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_grasping");
    
    ros::NodeHandle n;
    nodePtr = &n;
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
    
    ros::Subscriber gazebo_state_sub = n.subscribe("/gazebo/model_states",1,object_pose_callback);
    ros::Publisher gazebo_state_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1,true);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1,true );

    hand_synergy_pub["left_hand"] = n.advertise<trajectory_msgs::JointTrajectory>("/left_hand/joint_trajectory_controller/command",1,true);
    hand_synergy_pub["right_hand"] = n.advertise<trajectory_msgs::JointTrajectory>("/right_hand/joint_trajectory_controller/command",1,true);
    
    hand_actuated_joint["left_hand"] = "left_hand_synergy_joint";
    hand_actuated_joint["right_hand"] = "right_hand_synergy_joint";

    // do not take any object right now
    gotobject_count = 0;
    // clear action_count
    for(int i=0; i<10; i++)
      ros::spinOnce();
    action_count = 0;
    
    // create an object for grasping
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "";
    // the frame where the object position is considered (only when inserted, then it's *probably* fixed in the environment)
    attached_object.object.header.frame_id = "world";
    attached_object.object.id = "cylinder_testDB";
    
    // object pose
    geometry_msgs::Pose obj_pose;
    KDL::Frame world_obj(KDL::Vector(-0.5,-0.3,0.03));
    world_obj.M = KDL::Rotation::RPY(-M_PI/2.0,0.0,M_PI);
    tf::poseKDLToMsg(world_obj,obj_pose);
    
    // this will be interpreted as the object ID (to read in the DB)
    attached_object.weight = 1.0;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.30;
    primitive.dimensions[1] = 0.03;
    attached_object.object.primitives.push_back(primitive);
    // put the object floating on the table...
    obj_pose.position.z += 0.01;
    attached_object.object.primitive_poses.push_back(obj_pose);
    obj_pose.position.z -= 0.01;
    //I need to use this, though I wouldn't want : let me put it somewhere far...
    obj_pose.position.z -= 1;
    attached_object.object.mesh_poses.push_back(obj_pose);
    obj_pose.position.z += 1;
    
    gazebo_msgs::ModelState gazebo_msg;
    gazebo_msg.model_name = "cylinder";
    gazebo_msg.pose = obj_pose;
    gazebo_msg.pose.position.z += 1.0;
    gazebo_msg.reference_frame = "world";
    gazebo_state_pub.publish(gazebo_msg);
    ros::spinOnce();
    
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

    /////////////////////// get grasp data ///////////////////////
    
    std::string path = ros::package::getPath("dual_manipulation_ik_control");
    path.append("/test/grasp_data.txt");
    
    std::fstream fs;
    // fs.open (path, std::fstream::in | std::fstream::out | std::fstream::app*/);
    fs.open (path, std::fstream::in);
    if (fs.fail())
        return -1;
    
    double grasp_data[6];
    for (int i=0; i<6; ++i)
        fs >> grasp_data[i];
    
    fs.close();
    
    std::cout << "grasp_data: ";
    for (int i=0; i<6; ++i)
        std::cout << grasp_data[i] << " | ";
    std::cout << std::endl;
    
    KDL::Frame obj_hand;
    for (int i=0; i<3; i++)
      obj_hand.p.data[i] = grasp_data[i];
    
    obj_hand.M = KDL::Rotation::RPY(grasp_data[3],grasp_data[4],grasp_data[5]);
    
    tf::poseMsgToKDL(obj_pose,world_obj);
    
    /////////////////////// open the hand ///////////////////////
    std::vector<double > q = {0.0,0.0};
    std::vector<double > t = {0.0,1.0};
    
    std::string hand = "left_hand";
    
    if (!moveHand(hand,q,t))
      std::cout << "Moving the hand didn't work..." << std::endl;
    else
      std::cout << "Moving the hand worked!" << std::endl;

//     waitForHandMoved(hand,q.back());
    
    /////////////////////// plan to move the hand close to the object ///////////////////////
    srv.request.command = "plan";
    srv.request.ee_name = "left_hand";
    // compute hand pose
    geometry_msgs::Pose hand_pose;
    tf::poseKDLToMsg(world_obj*obj_hand,hand_pose);
    srv.request.ee_pose.clear();
    srv.request.ee_pose.push_back(hand_pose);
    
    std::cout << "hand_pose:" << std::endl << hand_pose << std::endl;

    // first, show hand_pose using a box
    publish_marker_utility(vis_pub,hand_pose);
// //     std::cout << "waiting to continue..." << std::endl;
// //     char tmp;
// //     std::cin >> tmp;
// // 
    action_count++;
    if (client.call(srv))
    {
	ROS_INFO("IK_control:test_grasping : %s request for %s accepted", srv.request.command.c_str(), srv.request.ee_name.c_str());
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
    while(action_count > 0)
    {
      std::cout << "Waiting for planning to complete... (action_count = " << action_count << ")" << std::endl;
      ros::spinOnce();
      usleep(200000);
    }

    /////////////////////// actually move the arm ///////////////////////
    srv.request.command = "execute";
    
    action_count++;
    if (client.call(srv))
    {
	ROS_INFO("IK_control:test_grasping : %s request for %s accepted", srv.request.command.c_str(), srv.request.ee_name.c_str());
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
    // sleep at least 1 second, just to be sure
    sleep(1);
    while(action_count > 0)
    {
      std::cout << "Waiting for execution to complete... (action_count = " << action_count << ")" << std::endl;
      ros::spinOnce();
      usleep(200000);
    }

    
    /////////////////////// perform grasp ///////////////////////

    // add a hand waypoint
    hand_pose.position.z -= 0.02;
    srv.request.ee_pose.push_back(hand_pose);

    // object to be grasped
    attached_object.link_name = "left_hand_palm_link";
    attached_object.object.header.frame_id = "left_hand_palm_link";
//     attached_object.object.header.stamp = ros::Time::now();
    // object pose relative to the palm: post-grasp pose from DB!
    obj_pose.position.x = 0.05; // primitive.dimensions.at(1); //+0.02;
    obj_pose.position.y = 0.0;
    obj_pose.position.z = 0.05;
    obj_pose.orientation.x = -0.707;
    obj_pose.orientation.y = 0.0;
    obj_pose.orientation.z = 0.0;
    obj_pose.orientation.w = 0.707;

    // // this does not work very well with the mesh (it gets oriented strangely)
    // rot.EulerZYX(0.0,0.0,M_PI/2.0).GetQuaternion(obj_pose.orientation.x,obj_pose.orientation.y,obj_pose.orientation.z,obj_pose.orientation.w);
    
    // attached_object.object.primitive_poses.clear();
    // attached_object.object.primitive_poses.push_back(obj_pose);
    attached_object.object.mesh_poses.clear();
    attached_object.object.mesh_poses.push_back(obj_pose);
    
    // hand joint trajectory
    trajectory_msgs::JointTrajectory grasp_traj;
//     grasp_traj.header.stamp = ros::Time::now();
    grasp_traj.joint_names.push_back(srv.request.ee_name + "_synergy_joint");
    q.clear();
    q.assign({0.0,0.4,1.0});
    t.clear();
    t.assign({0.0,0.4,1.0});
    
//     if (!moveHand(hand,q,t))
//       std::cout << "Moving the hand didn't work..." << std::endl;
    
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
    
    action_count++;
    if (client.call(srv))
    {
	ROS_INFO("IK_control:test_grasping : %s request for %s accepted", srv.request.command.c_str(), srv.request.ee_name.c_str());
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
    }
    
    while(action_count > 0)
    {
      std::cout << "Waiting for grasping to complete... (action_count = " << action_count << ")" << std::endl;
      ros::spinOnce();
      usleep(200000);
    }
    
    
//     /////////////////////// move the hand somewhere ///////////////////////
//     srv.request.command = "plan";
//     srv.request.ee_name = "left_hand";
//     hand_pose.position.y -= 0.35;
//     srv.request.ee_pose.clear();
//     srv.request.ee_pose.push_back(hand_pose);
//     
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK_control:test_grasping : %s request for %s accepted", srv.request.command.c_str(), srv.request.ee_name.c_str());
//     }
//     else
//     {
// 	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
//     
//     sleep(1);
//     
//     srv.request.command = "execute";
// 
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK_control:test_grasping : %s request for %s accepted", srv.request.command.c_str(), srv.request.ee_name.c_str());
//     }
//     else
//     {
// 	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
//     
//     sleep(5);
//     
//     /////////////////////// perform ungrasp ///////////////////////
//     srv.request.command = "ungrasp";
//     // leave the object where it is (w.r.t. the hand, right now), but detach it from the robot
// 
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK_control:test_grasping : %s object request accepted: %d", srv_obj.request.command.c_str(), (int)srv_obj.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
//     
//     sleep(5);
//     
//     /////////////////////// go back home ///////////////////////
//     srv.request.command = "home";
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK_control:test_grasping : %s object request accepted: %d", srv_obj.request.command.c_str(), (int)srv_obj.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
    
    // // get the final pose of the object
    // get the object from the scene, once
//     gotobject_count = 1;
//     while(gotobject_count > 0)
//     {
//       ros::spinOnce();
//       usleep(200000);
//     }
    
    for (int i=0; i<gazebo_mdl.name.size(); i++)
    {
      // std::cout << gazebo_mdl.name.at(i) << ":" << std::endl << gazebo_mdl.pose.at(i) << std::endl;
      if(gazebo_mdl.name.at(i)=="cylinder")
      {
	obj_pose = gazebo_mdl.pose.at(i);
	obj_pose.position.z -= 1.0;
      }
    }
    
    std::cout << "obj_pose from gazebo:" << std::endl;
    std::cout << obj_pose << std::endl;

    // clear stuff at the end
    for(int i=0; i<3; i++)
    {
      usleep(200000);
      ros::spinOnce();
    }

    return 0;
}