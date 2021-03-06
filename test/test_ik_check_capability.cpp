#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <iostream>
#include "ik_check_capability/ik_check_capability.h"

#define BIMANUAL 1
#define TEST_FIND_GROUP_IK 0
#define TEST_FIND_CLOSEST_GROUP_IK 0
#define TEST_CLIK 1

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> test_ik_check_capability "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_ik_check_capability");
    
    ros::NodeHandle n;
    ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1, true);
    moveit_msgs::DisplayRobotState display_rs_msg;
    
    // load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model_ = robot_model_loader.getModel();
    moveit::core::RobotState kinematic_state_(kinematic_model_);
    kinematic_state_.setToDefaultValues();
    
    std::vector<double> initial_guess;
    bool check_collisions = true;
    bool return_approximate_solution = true; //false;
    unsigned int attempts = 0;
    double timeout = 0.0;
    std::map <std::string, std::string > allowed_collisions = std::map< std::string, std::string >();
    allowed_collisions["left_hand_kuka_coupler_bottom"] = "left_arm_5_link";
    allowed_collisions["right_hand_kuka_coupler_bottom"] = "right_arm_5_link";

    dual_manipulation::ik_control::ikCheckCapability ik_check_capability(kinematic_model_);
    ik_check_capability.reset_robot_state("both_hands","dual_hand_arm_home");
    
    kinematic_state_ = ik_check_capability.get_robot_state();
    
    char y;
    std::cout << "Press any key to continue... ";
    std::cin >> y;
    
    ros::spinOnce();
    
    //visualize the default configuration (just to be sure it's been reset)
    robot_state::robotStateToRobotStateMsg(kinematic_state_, display_rs_msg.state);
    display_publisher.publish(display_rs_msg);
    
    sleep(1);
    
    std::vector<geometry_msgs::Pose> ee_poses;
    geometry_msgs::Pose ee_pose;
    
    ee_pose.position.x = -0.4;
    ee_pose.position.y = -0.1;
    ee_pose.position.z = +0.5;
    ee_pose.orientation.x = 0.0;
    ee_pose.orientation.y = 0.0;
    ee_pose.orientation.z = 0.707;
    ee_pose.orientation.w = 0.707;
    
    ee_poses.push_back(ee_pose);

    std::string ee_name = "left_hand";
    std::vector<std::vector<double>> solutions;

#ifndef BIMANUAL

#if TEST_FIND_GROUP_IK
    if(!ik_check_capability.find_group_ik(ee_name,ee_poses,solutions))
      ROS_ERROR_STREAM("Unable to solve ik request for " << ee_name << " in pose\n" << ee_pose);
    else
    {
      //visualize the result
      kinematic_state_ = ik_check_capability.get_robot_state();
      robot_state::robotStateToRobotStateMsg(kinematic_state_, display_rs_msg.state);
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
      //visualize the result
      kinematic_state_ = ik_check_capability.get_robot_state();
      robot_state::robotStateToRobotStateMsg(kinematic_state_, display_rs_msg.state);
      display_publisher.publish(display_rs_msg);
    }
#endif // TEST_FIND_GROUP_IK

#else
    ee_pose.position.y = +0.1;
    ee_pose.orientation.w = -0.707;
    ee_poses.push_back(ee_pose);
    ee_name = "both_hands";

#if TEST_FIND_GROUP_IK
    if(!ik_check_capability.find_group_ik(ee_name,ee_poses,solutions,initial_guess,check_collisions,false,attempts,timeout,allowed_collisions))
      ROS_ERROR_STREAM("Unable to solve ik request for " << ee_name << " in pose (1st one only shown here)\n" << ee_poses.at(0));
    else
    {
      //visualize the result
      kinematic_state_ = ik_check_capability.get_robot_state();
      robot_state::robotStateToRobotStateMsg(kinematic_state_, display_rs_msg.state);
      display_publisher.publish(display_rs_msg);
      
      sleep(1);
    }
    
    if(!ik_check_capability.find_group_ik(ee_name,ee_poses,solutions,initial_guess,check_collisions,return_approximate_solution,attempts,timeout,allowed_collisions))
      ROS_ERROR_STREAM("Unable to solve approximate ik request for " << ee_name << " in pose (1st one only shown here)\n" << ee_poses.at(0));
    else
    {
      //visualize the result
      kinematic_state_ = ik_check_capability.get_robot_state();
      robot_state::robotStateToRobotStateMsg(kinematic_state_, display_rs_msg.state);
      display_publisher.publish(display_rs_msg);
      
      sleep(1);
    }
#endif // TEST_FIND_GROUP_IK
    
#if TEST_FIND_CLOSEST_GROUP_IK
    moveit::core::JointModelGroup* jmg = kinematic_model_->getJointModelGroup("dual_hand_arm");
    const std::vector <std::string> ee_names = jmg->getAttachedEndEffectorNames();
    std::string pos_name("dual_hand_arm_home");
    kinematic_state_.setToDefaultValues(jmg,pos_name);
    // kinematic_state_.copyJointGroupPositions(jmg,initial_guess);
    kinematic_state_.setToDefaultValues();
    ik_check_capability.reset_robot_state();
    
    std::vector<std::pair<double,std::vector<std::vector<double>>>> it_info;
    bool store_iterations = true; // false;
    bool result;
    double allowed_distance = 0.001; //0.5;
    unsigned int trials_nr = 10; // 5;

    result = ik_check_capability.find_closest_group_ik(ee_name,ee_poses,solutions,it_info,store_iterations,allowed_distance,trials_nr,initial_guess,check_collisions,return_approximate_solution,attempts,timeout,allowed_collisions);
    
    if(!result)
      ROS_WARN_STREAM("Unable to solve ik request for " << ee_name << " with the required accuracy");
    
    std::cout << "Found " << it_info.size() << " solutions with distance from initial_guess of, respectively : " << std::endl;
    for(auto& it:it_info)
      std::cout << it.first << " | ";
    std::cout << std::endl;
    
    if(!it_info.empty())
    {
      std::cout << "Now displaying each solution found" << std::endl;
      for(auto& it:it_info)
      {
	std::cout << "Visualizing configuration at distance " << it.first << " <| | ";
	
	for(int i=0; i<ee_names.size(); i++)
	{
	  moveit::core::JointModelGroup* jm_subg = kinematic_model_->getEndEffector(ee_names.at(i));
	  kinematic_state_.setJointGroupPositions(jm_subg,it.second.at(i));
	  for(auto& q:it.second.at(i))
	    std::cout << q << " | ";
	}
	std::cout << "|>" << std::endl;

	//visualize the result
	robot_state::robotStateToRobotStateMsg(kinematic_state_, display_rs_msg.state);
	display_publisher.publish(display_rs_msg);
	
	sleep(1);
      }
      
      std::cout << "Now visualizing best configuration found! <| | ";
      for(int i=0; i<ee_names.size(); i++)
      {
	moveit::core::JointModelGroup* jm_subg = kinematic_model_->getEndEffector(ee_names.at(i));
	kinematic_state_.setJointGroupPositions(jm_subg,solutions.at(i));
	for(auto& q:solutions.at(i))
	  std::cout << q << " | ";
      }
      std::cout << "|>" << std::endl;

      //visualize the result
      robot_state::robotStateToRobotStateMsg(kinematic_state_, display_rs_msg.state);
      display_publisher.publish(display_rs_msg);

      sleep(1);
    }
#endif // TEST_FIND_CLOSEST_GROUP_IK

#if TEST_CLIK
    double clik_res = ik_check_capability.clik(ee_name,ee_poses,solutions,initial_guess,check_collisions,attempts,timeout,allowed_collisions);
    ROS_INFO_STREAM("ik_check_capability.clik was able to go through " << clik_res*100 << "% of the required motion");
    if(clik_res == 0)
      ROS_ERROR_STREAM("Unable to solve ik request for " << ee_name << " in pose (1st one only shown here)\n" << ee_poses.at(0));
    else
    {
      //visualize the result
      kinematic_state_ = ik_check_capability.get_robot_state();
      robot_state::robotStateToRobotStateMsg(kinematic_state_, display_rs_msg.state);
      display_publisher.publish(display_rs_msg);
      
      sleep(1);
    }
#endif // TEST_CLIK
    
#endif // #ifndef BIMANUAL

    for(int i=0; i<5; i++)
    {
      ros::spinOnce();
      usleep(100000);
    }

    return 0;
}