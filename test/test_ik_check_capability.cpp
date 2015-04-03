#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/robot_state.h>
#include <iostream>
#include "dual_manipulation_ik_control_ik_check/ik_check_capability.h"

#define BIMANUAL 1

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> test_ik_check_capability "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_ik_check_capability");
    
    ros::NodeHandle n;
    ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1, true);
    moveit_msgs::DisplayRobotState display_rs_msg;
    
    dual_manipulation::ik_control::ikCheckCapability ik_check_capability;
    
    char y;
    std::cout << "Press any key to continue... ";
    std::cin >> y;
    
//     for(int i=0; i<3; i++)
      ros::spinOnce();
    
    // load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model_ = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
    kinematic_state_->setToDefaultValues();
    
    display_rs_msg.state.joint_state.name = kinematic_state_->getVariableNames();
    
    std::cout << "Number of variables in the current robot state: " << display_rs_msg.state.joint_state.name.size() << std::endl;
    
    //visualize the default configuration (just to be sure it's been reset)
    for(int i=0; i<display_rs_msg.state.joint_state.name.size(); i++)
      display_rs_msg.state.joint_state.position.push_back(kinematic_state_->getVariablePosition(i));

    display_publisher.publish(display_rs_msg);
    
    sleep(1);
    
    std::vector<geometry_msgs::Pose> ee_poses;
    geometry_msgs::Pose ee_pose;
    
    ee_pose.position.x = -0.4;
    ee_pose.position.y = -0.1+0.1;
    ee_pose.position.z = +0.5;
    ee_pose.orientation.x = 0.0;
    ee_pose.orientation.y = 0.0;
    ee_pose.orientation.z = 0.707;
    ee_pose.orientation.w = 0.707;
    
    ee_poses.push_back(ee_pose);

    std::string ee_name = "left_hand";
    std::vector<std::vector<double>> solutions;

#ifndef BIMANUAL
    if(!ik_check_capability.find_group_ik(ee_name,ee_poses,solutions))
      ROS_ERROR_STREAM("Unable to solve ik request for " << ee_name << " in pose\n" << ee_pose);
    else
    {
      moveit::core::JointModelGroup* jmg = kinematic_model_->getEndEffector(ee_name);
      kinematic_state_->setJointGroupPositions(jmg,solutions.at(0));

      //visualize the result
      display_rs_msg.state.joint_state.position.clear();
      for(int i=0; i<display_rs_msg.state.joint_state.name.size(); i++)
	display_rs_msg.state.joint_state.position.push_back(kinematic_state_->getVariablePosition(i));

      display_publisher.publish(display_rs_msg);

      sleep(1);
    }
    
    ee_pose.position.y = +0.1;
    ee_pose.orientation.w = -0.707;
    ee_poses.clear();
    ee_poses.push_back(ee_pose);

    ee_name = "right_hand";
    
    if(!ik_check_capability.find_group_ik(ee_name,ee_poses,solutions))
      ROS_ERROR_STREAM("Unable to solve ik request for " << ee_name << " in pose\n" << ee_pose);
    else
    {
      moveit::core::JointModelGroup* jmg = kinematic_model_->getEndEffector(ee_name);
      kinematic_state_->setJointGroupPositions(jmg,solutions.at(0));

      //visualize the result
      display_rs_msg.state.joint_state.position.clear();
      for(int i=0; i<display_rs_msg.state.joint_state.name.size(); i++)
	display_rs_msg.state.joint_state.position.push_back(kinematic_state_->getVariablePosition(i));

      display_publisher.publish(display_rs_msg);
    }
#else
    ee_pose.position.y = +0.1-0.1;
    ee_pose.orientation.w = -0.707;
    ee_poses.push_back(ee_pose);
    ee_name = "both_hands";

    if(!ik_check_capability.find_group_ik(ee_name,ee_poses,solutions))
      ROS_ERROR_STREAM("Unable to solve ik request for " << ee_name << " in pose (1st one only shown here)\n" << ee_poses.at(0));
    else
    {
      moveit::core::JointModelGroup* jmg = kinematic_model_->getJointModelGroup("dual_hand_arm");
      const std::vector <std::string> ee_names = jmg->getAttachedEndEffectorNames();
      for(int i=0; i<ee_names.size(); i++)
      {
	moveit::core::JointModelGroup* jm_subg = kinematic_model_->getEndEffector(ee_names.at(i));
	kinematic_state_->setJointGroupPositions(jm_subg,solutions.at(i));
      }

      //visualize the result
      display_rs_msg.state.joint_state.position.clear();
      for(int i=0; i<display_rs_msg.state.joint_state.name.size(); i++)
	display_rs_msg.state.joint_state.position.push_back(kinematic_state_->getVariablePosition(i));

      display_publisher.publish(display_rs_msg);
    }
#endif

    ros::spin();

    return 0;
}