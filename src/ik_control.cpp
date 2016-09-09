#include "ik_control.h"
#include "trajectory_utils.h"
#include <dual_manipulation_shared/parsing_utils.h>
#include <dual_manipulation_shared/ik_response.h>
#include "moveit/trajectory_execution_manager/trajectory_execution_manager.h"
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>
//#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <tf_conversions/tf_kdl.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/String.h>
#include <ros/console.h>

#define SIMPLE_GRASP 1
#define CLASS_NAMESPACE "ikControl::"
#define CLASS_LOGNAME "ikControl"
#define DEFAULT_MAX_PLANNING_ATTEMPTS 1
#define DEBUG 0
#define CLOSED_HAND 0.5
#define LOG_INFO 0 // decide whether to log at info or warning level

#define REFACTOR_OUT 1
#include "random_planning_capability.h"
#include <dual_manipulation_ik_control/robot_controller_interface.h>

using namespace dual_manipulation::ik_control;
ros::Duration total_time;

ikControl::ikControl()
{
    #if !LOG_INFO
    // set logger level to warning only
    if( ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Warn) & ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn) )
        ros::console::notifyLoggerLevelsChanged();
    #endif
    // // to access a named logger (e.g. the one named with #define CLASS_LOGNAME) use the following syntax
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME "." CLASS_LOGNAME "_TIMING", ros::console::levels::Info) )
        ros::console::notifyLoggerLevelsChanged();
    
    setDefaultParameters();
    
    bool params_ok = node.getParam("ik_control_parameters", ik_control_params);
    
    if (params_ok)
    {
        sikm.sceneObjectManager_.reset(new sceneObjectManager());
        sikm.groupManager.reset(new GroupStructureManager(ik_control_params));
        parseParameters(ik_control_params);
    }
    
    setParameterDependentVariables();
}

void ikControl::reset()
{ 
    sikm.robotStateManager->reset_robot_state(sikm.planning_init_rs_,full_robot_group_,sikm.robotState_mutex_);
    movePlans_mutex_.lock();
    for(auto& plan:movePlans_){ move_group_interface::MoveGroup::Plan tmp_plan; std::swap(plan.second,tmp_plan);}
    movePlans_mutex_.unlock();
    sikm.end_time_mutex_.lock();
    sikm.movement_end_time_ = ros::Time::now();
    sikm.end_time_mutex_.unlock();
    
    sikm.map_mutex_.lock();
    sikm.grasped_obj_map_.clear();
    sikm.objects_map_.clear();
    sikm.map_mutex_.unlock();
    // for the first time, update the planning scene in full
    moveit_msgs::GetPlanningScene srv;
    uint32_t objects = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
    srv.request.components.components = objects;
    if(!scene_client_.call(srv))
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to call /get_planning_scene service - starting with an empty planning scene...");
    else
    {
        std::unique_lock<std::mutex> ul(sikm.map_mutex_);
        for(auto attObject:srv.response.scene.robot_state.attached_collision_objects)
        {
            for(auto links:allowed_collisions_)
                if(std::find(links.second.begin(),links.second.end(),attObject.link_name)!=links.second.end())
                {
                    sikm.grasped_obj_map_[links.first] = attObject.object.id;
                    sikm.objects_map_[attObject.object.id] = attObject;
                }
        }
    }
}

void ikControl::setDefaultParameters()
{
    // use global joint_states: this is NOT general, but it's how this worked till now
    joint_states_ = "/joint_states";
    position_threshold = 0.0007;
    velocity_threshold = 0.0007;
    hand_max_velocity = 2.0;
    hand_position_threshold = 1.0/200.0;
    epsilon_ = 0.001;
    
    kinematics_only_ = false;
    
    allowed_collision_prefixes_.clear();
    allowed_collision_prefixes_["left_hand"] = std::vector<std::string>({"left_hand","left_arm_7_link"});
    allowed_collision_prefixes_["right_hand"] = std::vector<std::string>({"right_hand","right_arm_7_link"});
    
    hand_synergy_pub_topics_.clear();
    hand_synergy_pub_topics_["left_hand"] = "/left_hand/joint_trajectory_controller/command";
    hand_synergy_pub_topics_["right_hand"] = "/right_hand/joint_trajectory_controller/command";
    
    controller_map_.clear();
    controller_map_["left_hand"] = "/left_arm/joint_trajectory_controller/follow_joint_trajectory/";
    controller_map_["right_hand"] = "/right_arm/joint_trajectory_controller/follow_joint_trajectory/";
    
    hand_actuated_joint_.clear();
    hand_actuated_joint_["left_hand"] = "left_hand_synergy_joint";
    hand_actuated_joint_["right_hand"] = "right_hand_synergy_joint";
    
    sikm.movement_end_time_ = ros::Time::now();
    
    trajectory_event_publisher_ = node.advertise<std_msgs::String>(trajectory_execution_manager::TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC, 1, false);
    scene_client_ = node.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    
    allowed_excursions_["left_hand"].clear();
    allowed_excursions_["left_hand"].assign({0.5,0.5,1.0,1.0,6.0,6.0,6.0});
    allowed_excursions_["right_hand"].clear();
    allowed_excursions_["right_hand"].assign({0.5,0.5,1.0,1.0,6.0,6.0,6.0});
    allowed_excursions_["both_hands"].clear();
    allowed_excursions_["both_hands"].assign({0.5,0.5,1.0,1.0,6.0,6.0,6.0,0.5,0.5,1.0,1.0,6.0,6.0,6.0});
    
    // apart from the first time, when this is done in the constructor after parameters are obtained from the server
    if(moveGroups_.size() > 0)
    {
        for(auto group:moveGroups_)
            delete group.second;
        moveGroups_.clear();
        movePlans_.clear();
        busy.clear();
        hand_pub.clear();
        hand_synergy_pub_.clear();
        delete ik_check_legacy_;
        
        setParameterDependentVariables();
    }
}

void ikControl::setParameterDependentVariables()
{
    // NOTE: this way, they never actually change - consider moving them in the constructor
    ros::NodeHandle n("~"); // a private NodeHandle is needed to set parameters for KDLKinematicsPlugin
    n.setParam("epsilon",epsilon_);
    robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
    robot_model_ = robot_model_loader_->getModel();
    moveit::planning_interface::MoveGroup::Options opt("full_robot");
    opt.robot_model_=robot_model_;
    //   opt.robot_description_="robot_description";
    //   opt.node_handle_=n;
    for(auto group_name:sikm.groupManager->get_group_map())
    {
        opt.group_name_=group_name.second;
        moveGroups_[group_name.first] = new move_group_interface::MoveGroup( opt, boost::shared_ptr<tf::Transformer>(), ros::Duration(5, 0) );
        movePlans_[group_name.first];
        
        for(auto capability:capabilities_.name)
        {
            busy[capabilities_.type.at(capability.first)][group_name.first] = false;
        }
    }
    
    for(auto capability:capabilities_.name)
    {
        hand_pub[capability.first] = node.advertise<dual_manipulation_shared::ik_response>("ik_control/" + capabilities_.msg.at(capability.first),1,this);
        ROS_DEBUG_STREAM("hand_pub[" << capability.second << "] => " + node.resolveName("ik_control",true) + "/" + capabilities_.msg.at(capability.first));
    }
    
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
        
        // JointTrajectory publishers
        if(hand_synergy_pub_topics_.count(chain_name))
            hand_synergy_pub_[chain_name] = node.advertise<trajectory_msgs::JointTrajectory>(hand_synergy_pub_topics_.at(chain_name),1,this);
    }
    
    // build robotModels and robotStates
    ik_check_legacy_ = new ikCheckCapability(robot_model_);
    sikm.planning_init_rs_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_));
    visual_rs_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_));
    
    // TODO: improve initialization of stuff to avoid having shared_ik_memory initialization all over the code
    bool full_robot_exists = sikm.groupManager->getGroupInSRDF("full_robot",full_robot_group_);
    assert(full_robot_exists);
    sikm.robotStateManager.reset(new RobotStateManager(robot_model_,joint_states_,full_robot_group_));
    sikm.robotController.reset(new RobotControllerInterface(ik_control_params,*(sikm.groupManager),joint_states_,*(sikm.robotStateManager),node));
    
    ikCheck_mutex_.lock();
    // TODO: check whether we need updated or not... I would say yes, and not including the AttachedCollisionObject in the robot state when wanting no collision checking
    planning_scene_ = ik_check_legacy_->get_planning_scene(true);
    ikCheck_mutex_.unlock();
    
    instantiateCapabilities();
    
    reset();
}

void ikControl::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    parseSingleParameter(params,position_threshold,"position_threshold");
    parseSingleParameter(params,velocity_threshold,"velocity_threshold");
    parseSingleParameter(params,hand_max_velocity,"hand_max_velocity");
    parseSingleParameter(params,hand_position_threshold,"hand_position_threshold");
    parseSingleParameter(params,kinematics_only_,"kinematics_only");
    parseSingleParameter(params,epsilon_,"epsilon");
    
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
    
    parseSingleParameter(params,hand_synergy_pub_topics_,"hand_synergy_pub_topics",chain_names);
    parseSingleParameter(params,controller_map_,"controller_map",chain_names);
    parseSingleParameter(params,hand_actuated_joint_,"hand_actuated_joint",chain_names);
}

bool ikControl::manage_object(dual_manipulation_shared::scene_object_service::Request& req)
{
    std::unique_lock<std::mutex> ul(sikm.scene_object_mutex_);
    // include allowed touch links, if any
    if((req.command == "attach") && (allowed_collisions_.count(req.ee_name)))
        req.attObject.touch_links.insert(req.attObject.touch_links.begin(),allowed_collisions_.at(req.ee_name).begin(),allowed_collisions_.at(req.ee_name).end());
    return sikm.sceneObjectManager_->manage_object(req);
}

void ikControl::ik_check_thread(dual_manipulation_shared::ik_service::Request req)
{
    ik_control_capabilities local_capability = ik_control_capabilities::IK_CHECK;
    
    ObjectLocker<std::mutex,bool> lkr(map_mutex_,busy.at(capabilities_.type.at(local_capability)).at(req.ee_name),false);
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Thread spawned! Computing IK for " << req.ee_name);
    
    dual_manipulation_shared::ik_response msg;
    msg.seq=req.seq;
    msg.group_name = req.ee_name;
    
    // NOTE: this lock is to perform both operations at the same time, but it's not necessary for thread-safety
    ikCheck_mutex_.lock();
    ik_check_legacy_->reset_robot_state();
    std::vector<double> sol;
    bool ik_ok = ik_check_legacy_->find_group_ik(req.ee_name,req.ee_pose.at(0),sol);
    ikCheck_mutex_.unlock();
    
    if(ik_ok)
    {
        msg.data = "done";
    }
    else
    {
        msg.data = "error";
    }
    hand_pub.at(local_capability).publish(msg); //publish on a topic when the IK check is done
    
    return;
}

void ikControl::planning_thread(dual_manipulation_shared::ik_service::Request req)
{
    ros::Time planning_start = ros::Time::now();
    std::string b="\033[0;34m";
    std::string n="\033[0m";
    
    ik_control_capabilities local_capability = capabilities_.from_name.at(req.command);
    
    ObjectLocker<std::mutex,bool> lkr(map_mutex_,busy.at(capabilities_.type.at(local_capability)).at(req.ee_name),false);
    
    if(!rndmPlan->canPerformCapability(local_capability))
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the requested planning capability is NOT implemented!!!");
        return;
    }
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Thread spawned! Computing plan for " << req.ee_name);
    
    dual_manipulation_shared::ik_response msg;
    bool plan_done(false);
    if(rndmPlan->canRun())
    {
        rndmPlan->performRequest(req);
        // multi-threaded case: while(!rndmPlan->isComplete()) usleep(5000);
        if(rndmPlan->isComplete())
            plan_done = rndmPlan->getResults(msg);
    }
    
    if(!plan_done)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the plan could NOT be obtained!");
        return;
    }
    
    hand_pub.at(local_capability).publish(msg); //publish on a topic when the trajectory is done
    
    total_time += (ros::Time::now() - planning_start);
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME + "_TIMING",b << CLASS_NAMESPACE << __func__ << " : This planning took [s]: " << (ros::Time::now() - planning_start).toSec() << n);
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME + "_TIMING",b << CLASS_NAMESPACE << __func__ << " : Duration till now [s]: " << total_time.toSec() << n);
    return;
}

void ikControl::execute_plan(dual_manipulation_shared::ik_service::Request req)
{
    ik_control_capabilities local_capability = ik_control_capabilities::MOVE;
    
    ObjectLocker<std::mutex,bool> lkr(map_mutex_,busy.at(capabilities_.type.at(local_capability)).at(req.ee_name),false);
    
    if(!trajExecute->canPerformCapability(local_capability))
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the requested capability is NOT implemented!!!");
        return;
    }
    
    dual_manipulation_shared::ik_response msg;
    bool done(false);
    if(trajExecute->canRun())
    {
        trajExecute->performRequest(req);
        // multi-threaded case: while(!trajExecute->isComplete()) usleep(5000);
        if(trajExecute->isComplete())
            done = trajExecute->getResults(msg);
    }
    
    if(!done)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the execution did NOT succeed!");
        return;
    }
    
    hand_pub.at(local_capability).publish(msg); //publish on a topic when done
    
    return;
}

bool ikControl::is_free_make_busy(std::string ee_name, std::string capability_name)
{
    std::unique_lock<std::mutex> ul(map_mutex_);
    
    if(!capabilities_.from_name.count(capability_name))
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unknown capability \"" << capability_name << "\", returning");
        return false;
    }
    
    ik_control_capability_types capability;
    capability = capabilities_.type.at(capabilities_.from_name.at(capability_name));
    
    if(!busy.at(capability).count(ee_name))
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unknown end effector \"" << ee_name << "\", returning");
        return false;
    }
    
    bool is_busy = false;
    
    // if I'm checking for a tree
    if(sikm.groupManager->is_tree(ee_name))
    {
        // if it's a capability which is not implemented yet for trees
        if(!capabilities_.implemented_for_trees.at(capability))
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Perform \"" << capability_name << "\" commands for each end-effector separately (tree version not implemented yet)! Returning");
            return false;
        }
        
        // check if any part of the tree is busy
        // NOTE: this is probably not necessary for all capabilities, but keep it coherent for now
        const std::vector<std::string>& chains = sikm.groupManager->get_tree_composition(ee_name);
        for(auto chain:chains)
            is_busy = is_busy || busy.at(capability).at(chain);
    }
    // if I'm checking for a chain, just be sure that its tree (if exists) is free
    // NOTE: if more than a single tree has that chain, the check should continue for all trees
    else
    {
        for(auto tree:sikm.groupManager->get_trees_with_chain(ee_name))
            is_busy = is_busy || busy.at(capability).at(tree);
    }
    
    // check whether the end-effector is free, and in case make it busy
    is_busy = is_busy || busy.at(capability).at(ee_name);
    if(!is_busy)
        busy.at(capability).at(ee_name) = true;
    else
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Already performing an ik_service of type " << capability_name << " for group " << ee_name);
    
    return (!is_busy);
}

bool ikControl::perform_ik(dual_manipulation_shared::ik_service::Request& req)
{
    std::cout << "perform_ik : req.command = " << req.command << std::endl;
    if(req.command == "stop")
    {
        sikm.robotController->stop();
        this->free_all();
        return true;
    }
    if(req.command == "free_all")
    {
        this->free_all();
        return true;
    }
    
    if(is_free_make_busy(req.ee_name,req.command))
    {
        std::thread* th;
        if(capabilities_.type.at(capabilities_.from_name.at(req.command)) == ik_control_capability_types::PLAN)
        {
            th = new std::thread(&ikControl::planning_thread,this, req);
        }
        else if(req.command == capabilities_.name.at(ik_control_capabilities::IK_CHECK))
        {
            th = new std::thread(&ikControl::ik_check_thread,this, req);
        }
        else if(req.command == capabilities_.name.at(ik_control_capabilities::MOVE))
        {
            th = new std::thread(&ikControl::execute_plan,this, req);
        }
        else if(req.command == capabilities_.name.at(ik_control_capabilities::HOME))
        {
            th = new std::thread(&ikControl::simple_homing,this, req);
        }
        else if(req.command == capabilities_.name.at(ik_control_capabilities::GRASP))
        {
            th = new std::thread(&ikControl::grasp,this, req);
        }
        else if(req.command == capabilities_.name.at(ik_control_capabilities::UNGRASP))
        {
            th = new std::thread(&ikControl::ungrasp,this, req);
        }
        else if(req.command == capabilities_.name.at(ik_control_capabilities::SET_TARGET))
        {
            this->add_target(req);
        }
        else if(req.command == capabilities_.name.at(ik_control_capabilities::SET_HOME_TARGET))
        {
            this->add_target(req);
        }
        else
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : this is strange - you shouldn't have come this far...!");
            return false;
        }
        used_threads_.push_back(th);
        return true;
    }
    
    return false;
}

ikControl::~ikControl()
{
    for(auto group:moveGroups_)
        delete group.second;
    
    for(int i=0; i<used_threads_.size(); i++)
        delete used_threads_.at(i);
    
    delete ik_check_legacy_;
    
    deleteCapabilities();
}

void ikControl::simple_homing(dual_manipulation_shared::ik_service::Request req)
{
    ik_control_capabilities local_capability = ik_control_capabilities::HOME;
    // TODO: remove this!!
    sikm.end_time_mutex_.lock();
    sikm.movement_end_time_ = ros::Time(0);
    sikm.end_time_mutex_.unlock();
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : going back home...");
    std::string ee_name=req.ee_name;
    std::string group_name;
    std::vector<std::string> chain_names;
    bool exists = sikm.groupManager->getGroupInSRDF(req.ee_name,group_name);
    assert(exists);
    if (sikm.groupManager->is_chain(ee_name))
        chain_names.push_back(ee_name);
    else
        chain_names = sikm.groupManager->get_tree_composition(ee_name);
    
    // also open the hand(s) we're moving home, but don't wait for it(them)
    std::vector <double > q = {0.0};
    std::vector <double > t = {1.0/hand_max_velocity};
    for(auto& ee:chain_names)
    {
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : opening hand " << ee);
        trajectory_msgs::JointTrajectory grasp_traj;
        sikm.robotController->moveHand(ee,q,t,grasp_traj);
    }
    
    // if the group is moving, stop it
    sikm.robotController->stop();
    // update planning_init_rs_ with current robot state
    bool target_ok = sikm.robotStateManager->reset_robot_state(sikm.planning_init_rs_,full_robot_group_,sikm.robotState_mutex_);
    
    dual_manipulation_shared::ik_serviceRequest ik_req;
    ik_req.command = capabilities_.name.at(ik_control_capabilities::SET_HOME_TARGET);
    ik_req.ee_name = "full_robot";
    perform_ik(ik_req);
    ik_req.command = capabilities_.name.at(ik_control_capabilities::PLAN);
    sikm.end_time_mutex_.lock();
    sikm.movement_end_time_ = ros::Time::now();
    sikm.end_time_mutex_.unlock();
    perform_ik(ik_req);
    usleep(500000);
    ik_control_capability_types capability;
    capability = capabilities_.type.at(ik_control_capabilities::PLAN);
    bool planning_done = false;
    while(!planning_done)
    {
        map_mutex_.lock();
        planning_done = !(busy.at(capability).at(ee_name));
        map_mutex_.unlock();
        if(!planning_done)
            usleep(100000);
    }
    
    // TODO: this message should follow from the planning and the moving results...
    dual_manipulation_shared::ik_response msg;
    msg.seq=req.seq;
    msg.group_name = req.ee_name;
    msg.data = "done";
    
    hand_pub.at(local_capability).publish(msg);
    map_mutex_.lock();
    busy.at(capabilities_.type.at(local_capability)).at(ee_name) = false;
    map_mutex_.unlock();
    
    ik_req.command = capabilities_.name.at(ik_control_capabilities::MOVE);
    perform_ik(ik_req);
    
    total_time = ros::Duration(0);
    std::string b="\033[0;34m";
    std::string n="\033[0m";
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME + "_TIMING",b << CLASS_NAMESPACE << __func__ << " : Duration reset! [s]: " << total_time.toSec() << n);
    
    return;
}

void ikControl::grasp(dual_manipulation_shared::ik_service::Request req)
{
    ik_control_capabilities local_capability = ik_control_capabilities::GRASP;
    
    ObjectLocker<std::mutex,bool> lkr(map_mutex_,busy.at(capabilities_.type.at(local_capability)).at(req.ee_name),false);
    
    if(!graspPlanExecute->canPerformCapability(local_capability))
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the requested capability is NOT implemented!!!");
        return;
    }
    
    dual_manipulation_shared::ik_response msg;
    bool done(false);
    if(graspPlanExecute->canRun())
    {
        graspPlanExecute->performRequest(req);
        // multi-threaded case: while(!graspPlanExecute->isComplete()) usleep(5000);
        if(graspPlanExecute->isComplete())
            done = graspPlanExecute->getResults(msg);
    }
    
    if(!done)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the execution did NOT succeed!");
        return;
    }
    
    hand_pub.at(local_capability).publish(msg); //publish on a topic when done
}

void ikControl::ungrasp(dual_manipulation_shared::ik_service::Request req)
{
    ik_control_capabilities local_capability = ik_control_capabilities::UNGRASP;
    
    ObjectLocker<std::mutex,bool> lkr(map_mutex_,busy.at(capabilities_.type.at(local_capability)).at(req.ee_name),false);
    
    if(!graspPlanExecute->canPerformCapability(local_capability))
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the requested capability is NOT implemented!!!");
        return;
    }
    
    dual_manipulation_shared::ik_response msg;
    bool done(false);
    if(graspPlanExecute->canRun())
    {
        graspPlanExecute->performRequest(req);
        // multi-threaded case: while(!graspPlanExecute->isComplete()) usleep(5000);
        if(graspPlanExecute->isComplete())
            done = graspPlanExecute->getResults(msg);
    }
    
    if(!done)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the execution did NOT succeed!");
        return;
    }
    
    hand_pub.at(local_capability).publish(msg); //publish on a topic when done
}

void ikControl::add_target(const dual_manipulation_shared::ik_service::Request& req)
{
    ik_control_capabilities local_capability = capabilities_.from_name.at(req.command);
    
    ObjectLocker<std::mutex,bool> lkr(map_mutex_,busy.at(capabilities_.type.at(local_capability)).at(req.ee_name),false);
    
    rndmPlan->add_target(req);
}

void ikControl::fillSharedMemory()
{
    // mutex for the shared variable
    std::unique_lock<std::mutex> lck11(sikm.m,std::defer_lock);
    std::unique_lock<std::mutex> lck12(sikm.planningScene_mutex_,std::defer_lock);
    std::unique_lock<std::mutex> lck13(sikm.movePlans_mutex_,std::defer_lock);
    // mutexes for ik_control variables
    std::unique_lock<std::mutex> lck21(scene_object_mutex_,std::defer_lock);
    std::unique_lock<std::mutex> lck22(movePlans_mutex_,std::defer_lock);
    
    // lock all together
    std::lock(lck11,lck12,lck13,lck21,lck22);
    
    // do actual assignment to shared memory
    sikm.planning_scene_ = planning_scene_;
    sikm.ik_control_params = &ik_control_params;
    sikm.movePlans_.swap(movePlans_);
}

void ikControl::instantiateCapabilities()
{
    fillSharedMemory();
    
    rndmPlan.reset(new randomPlanningCapability(sikm,node));
    trajExecute.reset(new TrajectoryExecutionCapability(sikm,node));
    graspPlanExecute.reset(new GraspingCapability(sikm,node));
}

void ikControl::deleteCapabilities()
{
    
}
