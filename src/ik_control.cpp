#include "ik_control.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <thread>

using namespace dual_manipulation::ik_control;

ikControl::ikControl()
{
    busy["left_hand"]=false;
    busy["right_hand"]=false;

    trj_gen["left_hand"] = new dual_manipulation::ik_control::trajectory_generator();
    trj_gen["right_hand"] = new dual_manipulation::ik_control::trajectory_generator();
    
    pub = node.advertise<std_msgs::String>("/ik_control/action_done",0,this);

}

void ikControl::ik_thread(dual_manipulation_shared::ik_service::Request req)
{
    KDL::Frame start_pose,final_pose;
    KDL::Frame next_pose;
    KDL::Twist next_vel;
    
    ROS_INFO("Thread spawned! Computing ik for %s",req.ee_name.c_str());
    
    //TODO: SENSE joints and COMPUTE ee cartesian position to set start
    
    start_pose.p = KDL::Vector::Zero();
    start_pose.M = KDL::Rotation::Identity();
    
    tf::poseMsgToKDL(req.ee_pose,final_pose);
    
    trj_gen[req.ee_name]->initialize_line_trajectory(req.time, start_pose, final_pose);
    
    //TODO: COMPUTE & PERFORM trajectory
    
    ros::Time start_t, now;
    ros::Duration final_t;
    
    final_t.fromNSec(req.time*1000000000.0);
    
    start_t = ros::Time::now();
    while(now-start_t<final_t)
    {
        now = ros::Time::now();
      
	trj_gen[req.ee_name]->line_trajectory(((now-start_t).toNSec()/1000000000.0), next_pose, next_vel);
	
	static tf::TransformBroadcaster br;
	tf::Transform current_robot_transform;
	tf::transformKDLToTF(next_pose,current_robot_transform);
	br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "base_link", req.ee_name.c_str()));
    }
  
    msg.data = "done";
    pub.publish(msg);
  
    busy[req.ee_name]=false;
    
    return;
}

bool ikControl::perform_ik(dual_manipulation_shared::ik_service::Request& req)
{
    if(!busy.count(req.ee_name))
    {
	ROS_ERROR("Unknown end effector %s, returning",req.ee_name.c_str());
	return false;
    }
    if(!busy[req.ee_name])
    {
	busy[req.ee_name]=true;
	std::thread* th = new std::thread(&ikControl::ik_thread,this, req);
    }
    else
    {
	ROS_WARN("Already performing a %s IK",req.ee_name.c_str());
	return false;
    }
    
    

    return true;
}

ikControl::~ikControl()
{
    delete trj_gen["left_hand"];
    delete trj_gen["right_hand"];
}
