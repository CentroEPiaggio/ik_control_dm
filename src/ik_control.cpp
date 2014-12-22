#include "ik_control.h"
#include "tf/tf.h"
#include <thread>

using namespace dual_manipulation::ik_control;

ikControl::ikControl()
{
    busy["left_hand"]=false;
    busy["right_hand"]=false;
}

void ikControl::ik_thread(dual_manipulation_shared::ik_service::Request req)
{
    KDL::Frame start_pose,final_pose;
    
    ROS_INFO("Thread spawned! Computing ik for %s",req.ee_name.c_str());
    
    //TODO: sense joints and compute ee cartesian position to set start
    
    start_pose.p = KDL::Vector::Zero();
    start_pose.M = KDL::Rotation::Identity();
    
    tf::poseMsgToKDL(req.ee_pose,final_pose);
    
    trj_gen.initialize_line_trajectory(req.time, start_pose, final_pose);
    
    //TODO: PERFORM initialize_line_trajectory
    
    //TODO: WRITE "done" or "error" ON TOPIC "action_done"
  
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

}
