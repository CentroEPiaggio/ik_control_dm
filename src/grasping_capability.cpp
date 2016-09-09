#include "grasping_capability.h"
#include <dual_manipulation_shared/parsing_utils.h>
//TODO: remove this when robotController will no longer depend on ik_shared_memory
#include <dual_manipulation_ik_control/robot_controller_interface.h>
#include "trajectory_utils.h"
#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetPlanningScene.h>

#define CLASS_NAMESPACE "GraspingCapability::"
#define CLASS_LOGNAME "GraspingCapability"
#define SIMPLE_GRASP 1
#define CLOSED_HAND 0.5

using namespace dual_manipulation::ik_control;

GraspingCapability::GraspingCapability(shared_ik_memory& sikm_, const ros::NodeHandle& node_) : sikm(sikm_), node(node_)
{
    reset();
}

GraspingCapability::~GraspingCapability() { }

bool GraspingCapability::canPerformCapability(const ik_control_capabilities& ik_capability) const
{
    if ((ik_capability == ik_control_capabilities::GRASP) || (ik_capability == ik_control_capabilities::UNGRASP))
        return true;
    
    return false;
}

void GraspingCapability::parseParameters(XmlRpc::XmlRpcValue& params)
{
    parseSingleParameter(params,hand_max_velocity,"hand_max_velocity");
    
    auto chain_names = sikm.groupManager->get_chains();
    // allowed collision parameters
    if(params.hasMember("allowed_collision_prefixes"))
    {
        std::map<std::string,std::vector<std::string>> acp_tmp;
        for(auto chain:chain_names)
        {
            parseSingleParameter(params["allowed_collision_prefixes"],acp_tmp[chain],chain);
            if(acp_tmp.at(chain).empty())
                acp_tmp.erase(chain);
        }
        if(!acp_tmp.empty())
        {
            allowed_collision_prefixes_.swap(acp_tmp);
            acp_tmp.clear();
        }
    }
    
    parseSingleParameter(params,hand_actuated_joint_,"hand_actuated_joint",chain_names);
}

void GraspingCapability::setParameterDependentVariables()
{
    robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
    robot_model_ = robot_model_loader_->getModel();
    ik_check_.reset(new ikCheckCapability(robot_model_));
    scene_client_ = node.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    
    for(auto chain_name:sikm.groupManager->get_chains())
    {
        // allowed touch links
        std::vector<std::string> links = robot_model_->getLinkModelNamesWithCollisionGeometry();
        for (auto link:links)
        {
            for (auto acpref:allowed_collision_prefixes_[chain_name])
                if(link.compare(0,acpref.size(),acpref.c_str()) == 0)
                {
                    allowed_collisions_[chain_name].push_back(link);
                    break;
                }
        }
    }
}

void GraspingCapability::reset()
{
    // set default and parameter-dependent variable value
    {
        std::unique_lock<std::mutex> ul(sikm.m);
        parseParameters(*(sikm.ik_control_params));
    }
    
    setParameterDependentVariables();
    
    busy.store(false);
}

void GraspingCapability::performRequest(dual_manipulation_shared::ik_serviceRequest req)
{
    // tell the interface I'm busy
    bool I_am_busy = busy.exchange(true);
    if(I_am_busy)
        return;
    
    ik_control_capabilities local_capability = capabilities_.from_name.at(req.command);
    if(ik_control_capabilities::GRASP == local_capability)
        grasp(req);
    else if(ik_control_capabilities::UNGRASP == local_capability)
        ungrasp(req);
    
    busy.store(false);
}

bool GraspingCapability::isComplete()
{
    return !busy.load();
}

bool GraspingCapability::canRun()
{
    return !busy.load();
}

bool GraspingCapability::getResults(dual_manipulation_shared::ik_response& res)
{
    // fill response message
    if(busy.load())
        return false;
    
    res = response_;
    return true;
}

void GraspingCapability::grasp(dual_manipulation_shared::ik_service::Request req)
{
    sikm.end_time_mutex_.lock();
    sikm.movement_end_time_ = ros::Time(0);
    sikm.end_time_mutex_.unlock();
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : \"" << req.attObject.object.id << "\" with \"" << req.ee_name << "\"");
    
    moveit::planning_interface::MoveItErrorCode error_code;
    
    response_.seq=req.seq;
    response_.group_name = req.ee_name;
    
    map_mutex_.lock();
    std::string grasped_obj;
    bool is_actuated_ee = hand_actuated_joint_.count(req.ee_name);
    map_mutex_.unlock();
    sikm.map_mutex_.lock();
    bool grasping = sikm.grasped_obj_map_.count(req.ee_name) != 0;
    if(grasping)
        grasped_obj = sikm.grasped_obj_map_.at(req.ee_name);
    sikm.map_mutex_.unlock();
    
    if(grasping)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : end-effector " << req.ee_name << " already grasped an object (obj id: " << grasped_obj << "), returning");
        response_.data = "error";
        // reset movement_end_time_ in order not to block planning
        sikm.end_time_mutex_.lock();
        sikm.movement_end_time_ = ros::Time::now();
        sikm.end_time_mutex_.unlock();
        return;
    }
    
    //TODO ATTENTION: this will not work for all non-actuated end-effector: should make this more general
    if(is_actuated_ee)
    {
        // // get timed trajectory from waypoints
        moveit_msgs::RobotTrajectory trajectory;
        std::string group_name;
        bool exists = sikm.groupManager->getGroupInSRDF(req.ee_name,group_name);
        assert(exists);
        double allowed_distance = 2.5;
        //ATTENTION: make this more general, depending on the robot
        std::vector<double> single_distances({0.5,0.5,0.5,1.0,2.0,2.0,2.0});
        std::lock(sikm.robotState_mutex_,ikCheck_mutex_);
        ik_check_->reset_robot_state(*(sikm.planning_init_rs_));
        sikm.robotState_mutex_.unlock();
        double completed = computeTrajectoryFromWPs(trajectory,req.ee_pose,*ik_check_,group_name,req.ee_name,false,allowed_distance,single_distances);
        ikCheck_mutex_.unlock();
        if(completed != 1.0)
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to get trajectory from waypoints, returning");
            response_.data = "error";
            // reset movement_end_time_ in order not to block planning
            sikm.end_time_mutex_.lock();
            sikm.movement_end_time_ = ros::Time::now();
            sikm.end_time_mutex_.unlock();
            return;
        }
        
        // // align trajectories in time and check hand velocity limits
        computeHandTiming(trajectory,req);
        
        // // do not fill the header if you're using different computers
        
        // // execution of approach
        moveit::planning_interface::MoveGroup::Plan movePlan;
        movePlan.trajectory_ = trajectory;
        error_code = sikm.robotController->asyncExecute(movePlan);
        if (error_code.val != 1)
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to send trajectory to the controller, returning");
            response_.data = "error";
            // reset movement_end_time_ in order not to block planning
            sikm.end_time_mutex_.lock();
            sikm.movement_end_time_ = ros::Time::now();
            sikm.end_time_mutex_.unlock();
            return;
        }
        
        #ifndef SIMPLE_GRASP
        moveHand(req.ee_name,req.grasp_trajectory);
        #endif
        
        // a good, planned trajectory has been successfully sent to the controller
        sikm.robotStateManager->reset_robot_state(sikm.planning_init_rs_,group_name,sikm.robotState_mutex_,trajectory);
        
        // // wait for approach
        if(!trajectory.joint_trajectory.points.empty())
        {
            std::unique_lock<std::mutex> ul(sikm.end_time_mutex_);
            sikm.movement_end_time_ = ros::Time::now() + trajectory.joint_trajectory.points.back().time_from_start;
        }
        bool good_stop = sikm.robotController->waitForExecution(req.ee_name,trajectory);
        // I didn't make it
        if (!good_stop)
        {
            sikm.robotStateManager->reset_robot_state(sikm.planning_init_rs_,group_name,sikm.robotState_mutex_);
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute approach trajectory, returning");
            response_.data = "error";
            return;
        }
        
        #ifdef SIMPLE_GRASP
        // // moveHand
        std::vector <double > q = {0.4,CLOSED_HAND};
        std::vector <double > t = {0.4/hand_max_velocity,0.5+1.0/hand_max_velocity};
        trajectory_msgs::JointTrajectory grasp_traj;
        // only wait if the hand actually exists
        if(sikm.robotController->moveHand(req.ee_name,q,t,grasp_traj))
            good_stop = sikm.robotController->waitForHandMoved(req.ee_name,q.back(),grasp_traj);
        #else
        // // wait for hand moved
        good_stop = waitForHandMoved(req.ee_name,req.grasp_trajectory.points.back().positions.at(0),req.grasp_trajectory);
        #endif
        // I didn't make it
        if (!good_stop)
        {
            sikm.robotStateManager->reset_robot_state(sikm.planning_init_rs_,group_name,sikm.robotState_mutex_);
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute grasp trajectory, returning");
            response_.data = "error";
            return;
        }
        
    }
    // // attach object (only if everything went smoothly)
    //check whether the object was present, and in case remove it from the environment
    dual_manipulation_shared::scene_object_service::Request req_obj;
    req_obj.command = "attach";
    req_obj.object_id = req.attObject.object.id;
    req_obj.attObject = req.attObject;
    // insert the links which does not constitute a collision
    req_obj.attObject.touch_links.insert(req_obj.attObject.touch_links.begin(),allowed_collisions_.at(req.ee_name).begin(),allowed_collisions_.at(req.ee_name).end());
    req_obj.object_db_id = req.object_db_id;
    
    sikm.scene_object_mutex_.lock();
    sikm.sceneObjectManager_->manage_object(req_obj);
    sikm.scene_object_mutex_.unlock();
    
    //ATTENTION: try to check for object in the scene: have they been set?
    bool object_attached = false;
    moveit_msgs::AttachedCollisionObject attObject_from_planning_scene;
    int attempts_left = 10;
    
    while(!object_attached && attempts_left-- > 0)
    {
        moveit_msgs::GetPlanningScene srv;
        uint32_t objects = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
        srv.request.components.components = objects;
        if(!scene_client_.call(srv))
            ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to call /get_planning_scene service - starting with an empty planning scene...");
        else
        {
            for(auto attObject:srv.response.scene.robot_state.attached_collision_objects)
                if(attObject.object.id == req.attObject.object.id)
                    if(std::find(allowed_collisions_.at(req.ee_name).begin(),allowed_collisions_.at(req.ee_name).end(),attObject.link_name) != allowed_collisions_.at(req.ee_name).end())
                    {
                        attObject_from_planning_scene = attObject;
                        object_attached = true;
                        break;
                    }
        }
        
        if(!object_attached)
        {
            ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : object \'" << req.attObject.object.id << "\' NOT FOUND in the planning scene (in the right place)!!! Sleeping 0.5s and checking again for " << attempts_left << " times...");
            usleep(500000);
        }
        else
            ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : object \'" << req.attObject.object.id << "\' FOUND in the planning scene (in the right place)!!!");
        
    }
    
    // we made it!
    response_.data = "done";
    sikm.map_mutex_.lock();
    for(auto& obj:sikm.grasped_obj_map_)
        if(obj.second == req.attObject.object.id)
        {
            sikm.grasped_obj_map_.erase(obj.first);
            break;
        }
        sikm.grasped_obj_map_[req.ee_name] = req.attObject.object.id;
    sikm.objects_map_[req.attObject.object.id] = attObject_from_planning_scene;
    sikm.map_mutex_.unlock();
    
    return;
}

void GraspingCapability::ungrasp(dual_manipulation_shared::ik_service::Request req)
{
    sikm.end_time_mutex_.lock();
    sikm.movement_end_time_ = ros::Time(0);
    sikm.end_time_mutex_.unlock();
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : \"" << req.attObject.object.id << "\" from \"" << req.ee_name << "\"");
    
    moveit::planning_interface::MoveItErrorCode error_code;
    
    response_.seq=req.seq;
    response_.group_name = req.ee_name;
    //NOTE: never check collision for waypoints (at least for now)
    bool check_collisions = false;
    double allowed_distance = 25;
    //ATTENTION: make this more general, depending on the robot
    std::vector<double> single_distances({0.5,0.5,0.5,1.0,1.0,1.0,1.0});
    
    // // get timed trajectory from waypoints
    moveit_msgs::RobotTrajectory trajectory;
    std::string group_name;
    bool exists = sikm.groupManager->getGroupInSRDF(req.ee_name,group_name);
    assert(exists);
    std::lock(sikm.robotState_mutex_,ikCheck_mutex_);
    ik_check_->reset_robot_state(*(sikm.planning_init_rs_));
    sikm.robotState_mutex_.unlock();
    double completed = computeTrajectoryFromWPs(trajectory,req.ee_pose,*ik_check_,group_name,req.ee_name,check_collisions, allowed_distance, single_distances);
    ikCheck_mutex_.unlock();
    
    bool good_stop = true;
    
    #ifndef SIMPLE_GRASP
    // // align trajectories in time and check hand velocity limits
    computeHandTiming(trajectory,req);
    // // moveHand
    moveHand(req.ee_name,req.grasp_trajectory);
    #elif SIMPLE_GRASP
    // // moveHand
    std::vector <double > q = {CLOSED_HAND, 0.0};
    std::vector <double > t = {0.0, 1.0/hand_max_velocity};
    trajectory_msgs::JointTrajectory grasp_traj;
    // only wait if the hand actually exists
    if(sikm.robotController->moveHand(req.ee_name,q,t,grasp_traj))
        good_stop = sikm.robotController->waitForHandMoved(req.ee_name,q.back(),grasp_traj);
    // I didn't make it
    if (!good_stop)
    {
        sikm.robotStateManager->reset_robot_state(sikm.planning_init_rs_,group_name,sikm.robotState_mutex_);
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute ungrasp trajectory, returning");
        response_.data = "error";
        // reset movement_end_time_ in order not to block planning
        sikm.end_time_mutex_.lock();
        sikm.movement_end_time_ = ros::Time::now();
        sikm.end_time_mutex_.unlock();
        return;
    }
    #endif
    
    sikm.map_mutex_.lock();
    std::string grasped_obj;
    bool grasping = sikm.grasped_obj_map_.count(req.ee_name) != 0;
    if(grasping)
        grasped_obj = sikm.grasped_obj_map_.at(req.ee_name);
    sikm.map_mutex_.unlock();
    
    if(grasping && (grasped_obj == req.attObject.object.id))
    {
        sikm.map_mutex_.lock();
        sikm.grasped_obj_map_.erase(req.ee_name);
        sikm.objects_map_.erase(grasped_obj);
        sikm.map_mutex_.unlock();
        
        // put the object back in the scene
        dual_manipulation_shared::scene_object_service::Request req_scene;
        req_scene.command = "detach";
        req_scene.attObject = req.attObject;
        req_scene.object_id = req.attObject.object.id;
        req_scene.object_db_id = req.object_db_id;
        
        sikm.scene_object_mutex_.lock();
        bool ok = sikm.sceneObjectManager_->manage_object(req_scene);
        sikm.scene_object_mutex_.unlock();
        if(!ok)
        {
            ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : object with ID \"" << req.attObject.object.id << "\" is not grasped by \"" << req.ee_name << "\". Performing ungrasp action anyway");
        }
    }
    else
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : end-effector " << req.ee_name << " has grasped nothing or a different object than " << req.attObject.object.id << ", not detaching it in the planning scene");
    
    if(completed > 0.0)
    {
        // // execution of retreat
        moveit::planning_interface::MoveGroup::Plan movePlan;
        movePlan.trajectory_ = trajectory;
        error_code = sikm.robotController->asyncExecute(movePlan);
        if (error_code.val != 1)
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to send trajectory to the controller, returning");
            response_.data = "error";
            // reset movement_end_time_ in order not to block planning
            sikm.end_time_mutex_.lock();
            sikm.movement_end_time_ = ros::Time::now();
            sikm.end_time_mutex_.unlock();
            return;
        }
        
        // a good, planned trajectory has been successfully sent to the controller
        sikm.robotStateManager->reset_robot_state(sikm.planning_init_rs_,group_name,sikm.robotState_mutex_,trajectory);
        
        // // wait for retreat
        if(!trajectory.joint_trajectory.points.empty())
        {
            std::unique_lock<std::mutex> ul(sikm.end_time_mutex_);
            sikm.movement_end_time_ = ros::Time::now() + trajectory.joint_trajectory.points.back().time_from_start;
        }
        good_stop = sikm.robotController->waitForExecution(req.ee_name,trajectory);
        // I didn't make it
        if (!good_stop)
        {
            sikm.robotStateManager->reset_robot_state(sikm.planning_init_rs_,group_name,sikm.robotState_mutex_);
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute retreat trajectory, returning");
            response_.data = "error";
            return;
        }
    }
    else
    {
        sikm.end_time_mutex_.lock();
        sikm.movement_end_time_ = ros::Time::now();
        sikm.end_time_mutex_.unlock();
    }
    
    #ifndef SIMPLE_GRASP
    // // wait for hand moved
    good_stop = waitForHandMoved(req.ee_name,req.grasp_trajectory.points.back().positions.at(0),req.grasp_trajectory);
    // I didn't make it
    if (!good_stop)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute ungrasp trajectory, returning");
        msg.data = "error";
        hand_pub.at(local_capability).publish(msg);
        return;
    }
    #endif
    
    response_.data = "done";
    
    return;
}