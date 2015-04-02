#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/robot_state.h>
#include <iostream>
#include "dual_manipulation_ik_control_ik_check/ik_check_capability.h"

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
    ee_poses.resize(1);
    geometry_msgs::Pose& ee_pose = ee_poses.at(0);
    
    // obtained via direct kinematics: left_hand_palm_link
    // pos [x y z]: 8.55678e-08 1.1292 1.06425
    // orient [x y z w]: -0.433013 0.25 0.433013 0.75
//     ee_pose.position.x = 0.2;
//     ee_pose.position.y = -0.2+1.1292;
//     ee_pose.position.z = -0.2+1.06425;
    ee_pose.position.x = -0.4;
    ee_pose.position.y = +0.0;
    ee_pose.position.z = +0.5;
    ee_pose.orientation.x = 0.0; //-0.433013;
    ee_pose.orientation.y = 0.0; //0.25;
    ee_pose.orientation.z = 0.707; //0.433013;
    ee_pose.orientation.w = -0.707; //0.75;

    std::string ee_name = "right_hand";
    std::vector<std::vector<double>> solutions;
    solutions.resize(1);

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

//     ee_pose.position.x = -0.2;
//     ee_pose.position.y = 0.2-1.1292;
//     ee_pose.position.z = -0.2+1.06425;
    ee_pose.position.y = -0.0;
    ee_pose.orientation.w = 0.707;

    ee_name = "left_hand";
    solutions.at(0).clear();

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

    ros::spin();

    return 0;
}