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
    
    if (node.getParam("ik_control_parameters", ik_control_params))
        parseParameters(ik_control_params);
    
    setParameterDependentVariables();
}

void ikControl::reset()
{ 
    reset_robot_state(sikm.planning_init_rs_);
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
        std::unique_lock<std::mutex>(sikm.map_mutex_);
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
    chain_names_list_.clear();
    chain_names_list_.assign({"left_hand","right_hand"});
    tree_names_list_.clear();
    tree_names_list_.assign({"both_hands"});
    tree_composition_.clear();
    tree_composition_["both_hands"] = std::vector<std::string>({"left_hand","right_hand"});
    
    group_map_.clear();
    group_map_["left_hand"] = "left_hand_arm";
    group_map_["right_hand"] = "right_hand_arm";
    group_map_["both_hands"] = "dual_hand_arm";
    
    position_threshold = 0.0007;
    velocity_threshold = 0.0007;
    hand_max_velocity = 2.0;
    hand_position_threshold = 1.0/200.0;
    clik_threshold_ = 0.1;
    epsilon_ = 0.001;
    
    kinematics_only_ = false;
    
    allowed_collision_prefixes_.clear();
    allowed_collision_prefixes_["left_hand"] = std::vector<std::string>({"left_hand","left_arm_7_link"});
    allowed_collision_prefixes_["right_hand"] = std::vector<std::string>({"right_hand","right_arm_7_link"});
    
    // planner parameters
    planner_id_ = "RRTstarkConfigDefault";
    planning_time_ = 2.0;
    backup_planner_id_ = "RRTConnectkConfigDefault";
    backup_planning_time_ = 5.0;
    max_planning_attempts_ = DEFAULT_MAX_PLANNING_ATTEMPTS;
    backup_max_planning_attempts_ = DEFAULT_MAX_PLANNING_ATTEMPTS;
    goal_position_tolerance_ = 0.005;
    goal_orientation_tolerance_ = 0.005;
    goal_joint_tolerance_ = 0.005;
    ws_bounds_.assign({-1.2,-1.5,0.1,0.2,1.5,1.5});
    
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
    motionPlan_client_ = node.serviceClient<moveit_msgs::GetMotionPlan>(move_group::PLANNER_SERVICE_NAME);
    
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
        delete ik_check_;
        delete position_only_ik_check_;
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
    moveit::planning_interface::MoveGroup::Options opt(group_map_.begin()->second);
    opt.robot_model_=robot_model_;
    //   opt.robot_description_="robot_description";
    //   opt.node_handle_=n;
    for(auto group_name:group_map_)
    {
        opt.group_name_=group_name.second;
        moveGroups_[group_name.first] = new move_group_interface::MoveGroup( opt, boost::shared_ptr<tf::Transformer>(), ros::Duration(5, 0) );
        movePlans_[group_name.first];
        
        for(auto capability:capabilities_.name)
        {
            busy[capabilities_.type[capability.first]][group_name.first] = false;
        }
    }
    
    for(auto capability:capabilities_.name)
    {
        hand_pub[capability.first] = node.advertise<dual_manipulation_shared::ik_response>("ik_control/" + capabilities_.msg[capability.first],1,this);
        ROS_DEBUG_STREAM("hand_pub[" << capability.second << "] => " + node.resolveName("ik_control",true) + "/" + capabilities_.msg[capability.first]);
    }
    
    for(auto item:moveGroups_)
    {
        item.second->setPlannerId(backup_planner_id_);
        item.second->setPlanningTime(backup_planning_time_);
        item.second->setNumPlanningAttempts(backup_max_planning_attempts_);
        item.second->setGoalPositionTolerance(goal_position_tolerance_);
        item.second->setGoalOrientationTolerance(goal_orientation_tolerance_);
        item.second->setGoalJointTolerance(goal_joint_tolerance_);
        item.second->setWorkspace(ws_bounds_.at(0),ws_bounds_.at(1),ws_bounds_.at(2),ws_bounds_.at(3),ws_bounds_.at(4),ws_bounds_.at(5));
    }
    
    for(auto chain_name:chain_names_list_)
    {
        // allowed touch links
        std::vector<std::string> links = robot_model_->getLinkModelNamesWithCollisionGeometry();
        for (auto link:links)
            for (auto acpref:allowed_collision_prefixes_[chain_name])
                if(link.compare(0,acpref.size(),acpref.c_str()) == 0)
                {
                    allowed_collisions_[chain_name].push_back(link);
                    break;
                }
                
                // JointTrajectory publishers
                if(hand_synergy_pub_topics_.count(chain_name))
                    hand_synergy_pub_[chain_name] = node.advertise<trajectory_msgs::JointTrajectory>(hand_synergy_pub_topics_.at(chain_name),1,this);
    }
    
    // build robotModels and robotStates
    // set parameters of the private nodeHandle to load a robotModel with position only IK
    for(auto jmg:group_map_)
        n.setParam(jmg.second + "/position_only_ik",true);
    position_only_ik_robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
    position_only_ik_robot_model_ = position_only_ik_robot_model_loader_->getModel();
    for(auto jmg:group_map_)
        n.setParam(jmg.second + "/position_only_ik",false);
    ik_check_ = new ikCheckCapability(robot_model_);
    position_only_ik_check_ = new ikCheckCapability(position_only_ik_robot_model_);
    ik_check_legacy_ = new ikCheckCapability(robot_model_);
    target_rs_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_));
    sikm.planning_init_rs_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_));
    visual_rs_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_));
    
    // default constructor will read values from the ROS parameter server, which are loaded once we load move_group (see "omp_planning_pipeline.launch.xml")
    ros::NodeHandle move_group_node("move_group");
    ros::NodeHandle private_nh("~");
    if(node.hasParam("ik_control_parameters/fix_start_state_collision/jiggle_fraction"))
    {
        double jiggle_fraction;
        node.getParam("ik_control_parameters/fix_start_state_collision/jiggle_fraction",jiggle_fraction);
        private_nh.setParam("jiggle_fraction",jiggle_fraction);
    }
    if(node.hasParam("ik_control_parameters/fix_start_state_collision/max_sampling_attempts"))
    {
        double max_sampling_attempts;
        node.getParam("ik_control_parameters/fix_start_state_collision/max_sampling_attempts",max_sampling_attempts);
        private_nh.setParam("max_sampling_attempts",max_sampling_attempts);
    }
    robotState_mutex_.lock();
    pipeline_ = planning_pipeline::PlanningPipelinePtr(new planning_pipeline::PlanningPipeline(target_rs_->getRobotModel(),move_group_node,"planning_plugin","request_adapters"));
    robotState_mutex_.unlock();
    
    ikCheck_mutex_.lock();
    // TODO: check whether we need updated or not... I would say yes, and not including the AttachedCollisionObject in the robot state when wanting no collision checking
    planning_scene_ = ik_check_->get_planning_scene(true);
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
    parseSingleParameter(params,clik_threshold_,"clik_threshold");
    parseSingleParameter(params,epsilon_,"epsilon");
    
    parseSingleParameter(params,chain_names_list_,"chain_group_names",1);
    parseSingleParameter(params,tree_names_list_,"tree_group_names",1);
    
    // list of chains composing each tree
    if(params.hasMember("tree_composition"))
    {
        std::map<std::string,std::vector<std::string>> tc_tmp;
        for(auto tree:tree_names_list_)
        {
            parseSingleParameter(params["tree_composition"],tc_tmp[tree],tree);
            if(tc_tmp.at(tree).empty())
                tc_tmp.erase(tree);
        }
        if(!tc_tmp.empty())
        {
            tree_composition_.swap(tc_tmp);
            tc_tmp.clear();
        }
    }
    std::vector<std::string> tree_names_tmp;
    tree_names_tmp.swap(tree_names_list_);
    for(auto tree:tree_names_tmp)
        if(!tree_composition_.count(tree) || tree_composition_.at(tree).size() == 0)
            ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : No composition is specified for tree '" << tree << "': check the yaml configuration.");
        else
            tree_names_list_.push_back(tree);
        
        std::map<std::string,std::string> map_tmp,map_tmp_tree;
    parseSingleParameter(params,map_tmp,"group_map",chain_names_list_);
    parseSingleParameter(params,map_tmp_tree,"group_map",tree_names_list_);
    if(!map_tmp_tree.empty())
        for(auto tree:map_tmp_tree)
            map_tmp[tree.first] = tree.second;
        if(!map_tmp.empty())
        {
            group_map_.swap(map_tmp);
            map_tmp.clear();
        }
        
        // allowed collision parameters
        if(params.hasMember("allowed_collision_prefixes"))
        {
            std::map<std::string,std::vector<std::string>> acp_tmp;
            for(auto chain:chain_names_list_)
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
        
        parseSingleParameter(params,hand_synergy_pub_topics_,"hand_synergy_pub_topics",chain_names_list_);
        parseSingleParameter(params,controller_map_,"controller_map",chain_names_list_);
        parseSingleParameter(params,hand_actuated_joint_,"hand_actuated_joint",chain_names_list_);
        
        // planner parameters
        if(params.hasMember("motion_planner"))
        {
            parseSingleParameter(params["motion_planner"],planner_id_,"planner_id");
            parseSingleParameter(params["motion_planner"],planning_time_,"planning_time");
            parseSingleParameter(params["motion_planner"],backup_planner_id_,"backup_planner_id");
            parseSingleParameter(params["motion_planner"],backup_planning_time_,"backup_planning_time");
            parseSingleParameter(params["motion_planner"],max_planning_attempts_,"max_planning_attempts");
            parseSingleParameter(params["motion_planner"],backup_max_planning_attempts_,"backup_max_planning_attempts");
            if(max_planning_attempts_ <= 0) max_planning_attempts_ = DEFAULT_MAX_PLANNING_ATTEMPTS;
            if(backup_max_planning_attempts_ <= 0) backup_max_planning_attempts_ = DEFAULT_MAX_PLANNING_ATTEMPTS;
            parseSingleParameter(params["motion_planner"],goal_position_tolerance_,"goal_position_tolerance");
            parseSingleParameter(params["motion_planner"],goal_orientation_tolerance_,"goal_orientation_tolerance");
            parseSingleParameter(params["motion_planner"],goal_joint_tolerance_,"goal_joint_tolerance");
            parseSingleParameter(params["motion_planner"],ws_bounds_,"workspace_bounds",6);
        }
}

bool ikControl::manage_object(dual_manipulation_shared::scene_object_service::Request& req)
{
    std::unique_lock<std::mutex>(scene_object_mutex_);
    // include allowed touch links, if any
    if((req.command == "attach") && (allowed_collisions_.count(req.ee_name)))
        req.attObject.touch_links.insert(req.attObject.touch_links.begin(),allowed_collisions_.at(req.ee_name).begin(),allowed_collisions_.at(req.ee_name).end());
    return scene_object_manager_.manage_object(req);
}

bool ikControl::waitForHandMoved(std::string& hand, double hand_target, const trajectory_msgs::JointTrajectory& traj)
{
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : entered");
    
    // if an end-effector is not a hand
    if(!hand_actuated_joint_.count(hand))
        return true;
    
    if(kinematics_only_)
    {
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : kinematics_only execution - moving on after the trajectory has been shown");
        moveit_msgs::RobotTrajectory traj2;
        traj2.joint_trajectory = traj;
        publishTrajectoryPath(traj2);
        return true;
    }
    
    int counter = 0;
    int hand_index = 0;
    bool good_stop = false;
    sensor_msgs::JointStateConstPtr joint_states;
    
    joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",node,ros::Duration(3));
    for(auto joint:joint_states->name)
    {
        if(joint == hand_actuated_joint_.at(hand))
        {
            break;
        }
        hand_index++;
    }
    if(hand_index >= joint_states->name.size())
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : " << hand_actuated_joint_.at(hand) << " NOT found in /joint_states - returning");
        return false;
    }
    
    // wait for up to 10 more seconds
    while(counter<100)
    {
        //get joint states
        joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",node,ros::Duration(3));
        
        if(joint_states->name.at(hand_index) != hand_actuated_joint_.at(hand))
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : joints in joint_states changed order");
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
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : exiting with good_stop OK");
    else
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : exiting with error");
    return good_stop;
}

bool ikControl::waitForExecution(std::string ee_name, moveit_msgs::RobotTrajectory traj)
{
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : entered");
    
    if (traj.joint_trajectory.points.size() == 0)
    {
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the trajectory to wait for was empty");
        return true;
    }
    
    if(kinematics_only_)
    {
        sikm.end_time_mutex_.lock();
        sikm.movement_end_time_ = ros::Time::now() + traj.joint_trajectory.points.back().time_from_start;
        sikm.end_time_mutex_.unlock();
        publishTrajectoryPath(traj);
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : kinematics_only execution - moving on after the trajectory has been shown");
        return true;
    }
    
    map_mutex_.lock();
    int has_ctrl = controller_map_.count(ee_name);
    std::string controller_name;
    if(has_ctrl != 0)
        controller_name = controller_map_.at(ee_name);
    map_mutex_.unlock();
    
    control_msgs::FollowJointTrajectoryActionResultConstPtr pt;
    ros::Duration timeout = traj.joint_trajectory.points.back().time_from_start;
    sikm.end_time_mutex_.lock();
    sikm.movement_end_time_ = ros::Time::now() + timeout;
    sikm.end_time_mutex_.unlock();
    if(has_ctrl != 0)
    {
        // only do this if a controller exists - use a scaled timeout
        timeout = timeout*1.0;
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : waiting for at most " << timeout << " (trajectory total time)");
        pt = ros::topic::waitForMessage<control_msgs::FollowJointTrajectoryActionResult>(controller_name + "result",node,timeout);
        if(pt)
            ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : received message - error_code=" << pt->result.error_code);
        else
            ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : timeout reached");
    }
    else
    {
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : waiting for at least " << timeout << " (trajectory total time)");
        timeout.sleep();
    }
    
    std::vector<std::string> joints = traj.joint_trajectory.joint_names;
    std::vector<double> q,Dq,goal_q;
    q.reserve(joints.size());
    Dq.reserve(joints.size());
    goal_q.reserve(joints.size());
    
    // (remember that the goal position is the last pose of the trajectory)
    for(auto q_i:traj.joint_trajectory.points.back().positions)
        goal_q.push_back(q_i);
    
    // goal size and joints size MUST be equal: check
    assert(goal_q.size() == joints.size());
    
    double vel,dist;
    bool good_stop = false;
    
    sensor_msgs::JointStateConstPtr joint_states;
    
    // wait until the robot is moving
    vel = 1.0 + velocity_threshold;
    while(vel > velocity_threshold)
    {
        q.clear();
        Dq.clear();
        vel = 0.0;
        dist = 0.0;
        
        // TODO: use one subscriber instead of waiting on single messages?
        joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",node,ros::Duration(3));
        bool joint_found;
        
        //get joint states
        for(auto joint:joints)
        {
            joint_found = false;
            for(int i=0; i<joint_states->name.size(); i++)
            {
                if(joint == joint_states->name.at(i))
                {
                    q.push_back(joint_states->position.at(i));
                    Dq.push_back(joint_states->velocity.at(i));
                    joint_found = true;
                    break;
                }
            }
            if(!joint_found)
            {
                ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : couldn't find requested joints!");
                return false;
            }
        }
        
        for(auto v:Dq)
            vel += std::norm(v);
        
        //if velocity < eps1
        if (vel<velocity_threshold)
        {
            for(int i=0; i<q.size(); i++)
                dist += std::norm(q.at(i) - goal_q.at(i));
            
            //if norm(fk(position) - goal) < eps2
            if (dist<position_threshold)
            {
                good_stop = true;
                break;
            }
            else
            {
                ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : vel=" << vel << " (< " << velocity_threshold << ") but dist=" << dist << " (>= " << position_threshold << ")");
                break;
            }
        }
        usleep(100000);
    }
    
    if(good_stop)
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : exiting with good_stop OK");
    else
    {
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : exiting with error");
        reset();
    }
    return good_stop;
}

void ikControl::ik_check_thread(dual_manipulation_shared::ik_service::Request req)
{
    ik_control_capabilities local_capability = ik_control_capabilities::IK_CHECK;
    
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
    
    map_mutex_.lock();
    busy.at(capabilities_.type.at(local_capability)).at(req.ee_name)=false;
    map_mutex_.unlock();
    
    return;
}

void ikControl::planning_thread(dual_manipulation_shared::ik_service::Request req)
{
    ros::Time planning_start = ros::Time::now();
    std::string b="\033[0;34m";
    std::string n="\033[0m";
    
    ik_control_capabilities local_capability = capabilities_.from_name.at(req.command);
    
    if(!rndmPlan->canPerformCapability(local_capability))
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : the requested planning capability is NOT implemented!!!");
        map_mutex_.lock();
        busy.at(capabilities_.type.at(local_capability)).at(req.ee_name)=false;
        map_mutex_.unlock();
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
        map_mutex_.lock();
        busy.at(capabilities_.type.at(local_capability)).at(req.ee_name)=false;
        map_mutex_.unlock();
        return;
    }
    
    // NOTE: planning specific stuff: update planning_init_rs_ with trajectory last waypoint...
    sikm.movePlans_mutex_.lock();
    moveit_msgs::RobotTrajectory traj = sikm.movePlans_.at(req.ee_name).trajectory_;
    sikm.movePlans_mutex_.unlock();
    reset_robot_state(sikm.planning_init_rs_,req.ee_name,traj);
    
    hand_pub.at(local_capability).publish(msg); //publish on a topic when the trajectory is done
    
    map_mutex_.lock();
    busy.at(capabilities_.type.at(local_capability)).at(req.ee_name)=false;
    map_mutex_.unlock();
    
    total_time += (ros::Time::now() - planning_start);
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME + "_TIMING",b << CLASS_NAMESPACE << __func__ << " : This planning took [s]: " << (ros::Time::now() - planning_start).toSec() << n);
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME + "_TIMING",b << CLASS_NAMESPACE << __func__ << " : Duration till now [s]: " << total_time.toSec() << n);
    
    return;
}

void ikControl::execute_plan(dual_manipulation_shared::ik_service::Request req)
{
    ik_control_capabilities local_capability = ik_control_capabilities::MOVE;
    sikm.end_time_mutex_.lock();
    sikm.movement_end_time_ = ros::Time(0);
    sikm.end_time_mutex_.unlock();
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Executing plan for " << req.ee_name);
    
    moveit::planning_interface::MoveItErrorCode error_code;
    moveit::planning_interface::MoveGroup::Plan movePlan;
    
    sikm.movePlans_mutex_.lock();
    //NOTE: to be sure that no other execution is tried using this movePlan, use swap
    std::swap(movePlan,sikm.movePlans_.at(req.ee_name));
    sikm.movePlans_mutex_.unlock();
    
    // old execution method: does not allow for two trajectories at the same time
    moveGroups_mutex_.lock();
    if(!kinematics_only_)
        error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlan);
    moveGroups_mutex_.unlock();
    
    bool good_stop = waitForExecution(req.ee_name,movePlan.trajectory_);
    
    dual_manipulation_shared::ik_response msg;
    msg.seq=req.seq;
    msg.group_name = req.ee_name;
    
    if(good_stop)
    {
        msg.data = "done";
    }
    else
    {
        msg.data = "error";
    }
    hand_pub.at(local_capability).publish(msg); //publish on a topic when the trajectory is done
    
    map_mutex_.lock();
    busy.at(capabilities_.type.at(local_capability)).at(req.ee_name)=false;
    map_mutex_.unlock();
    
    return;
}

bool ikControl::is_free_make_busy(std::string ee_name, std::string capability_name)
{
    std::unique_lock<std::mutex>(map_mutex_);
    
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
    if(std::find(tree_names_list_.begin(),tree_names_list_.end(),ee_name) != tree_names_list_.end())
    {
        // if it's a capability which is not implemented yet for trees
        if(!capabilities_.implemented_for_trees.at(capability))
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Perform \"" << capability_name << "\" commands for each end-effector separately (tree version not implemented yet)! Returning");
            return false;
        }
        
        // check if any part of the tree is busy
        // TODO: this is probably not necessary for all capabilities, but keep it coherent for now
        std::vector<std::string>& chains = tree_composition_.at(ee_name);
        for(auto chain:chains)
            is_busy = is_busy || busy.at(capability).at(chain);
    }
    // if I'm checking for a chain, just be sure that its tree (if exists) is free
    // TODO: this is probably not necessary for all capabilities, but keep it coherent for now
    // NOTE: if more than a single tree has that chain, the check should continue for all trees
    else
    {
        for(auto tree:tree_names_list_)
            if(std::find(tree_composition_.at(tree).begin(),tree_composition_.at(tree).end(),ee_name) != tree_composition_.at(tree).end())
            {
                is_busy = is_busy || busy.at(capability).at(tree);
                continue;
            }
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
        this->stop();
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
        else if(req.command == capabilities_.name[ik_control_capabilities::IK_CHECK])
        {
            th = new std::thread(&ikControl::ik_check_thread,this, req);
        }
        else if(req.command == capabilities_.name[ik_control_capabilities::MOVE])
        {
            th = new std::thread(&ikControl::execute_plan,this, req);
        }
        else if(req.command == capabilities_.name[ik_control_capabilities::HOME])
        {
            th = new std::thread(&ikControl::simple_homing,this, req);
        }
        else if(req.command == capabilities_.name[ik_control_capabilities::GRASP])
        {
            th = new std::thread(&ikControl::grasp,this, req);
        }
        else if(req.command == capabilities_.name[ik_control_capabilities::UNGRASP])
        {
            th = new std::thread(&ikControl::ungrasp,this, req);
        }
        else if(req.command == capabilities_.name[ik_control_capabilities::SET_TARGET])
        {
            this->add_target(req);
        }
        else if(req.command == capabilities_.name[ik_control_capabilities::SET_HOME_TARGET])
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
    
    delete ik_check_;
    delete position_only_ik_check_;
    delete ik_check_legacy_;
    
    deleteCapabilities();
}

bool ikControl::moveHand(std::string& hand, std::vector< double >& q, std::vector< double >& t, trajectory_msgs::JointTrajectory& grasp_traj)
{
    // if an end-effector is not a hand
    if(!hand_actuated_joint_.count(hand))
        return true;
    // // do not fill the header if you're using different computers
    
    grasp_traj.joint_names.push_back(hand_actuated_joint_.at(hand));
    
    if (t.size() != q.size())
    {
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : timing vector size non compatible with joint vector size, using a default timing of 1 second");
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
    
    hand_synergy_pub_mutex_.lock();
    hand_synergy_pub_.at(hand).publish(grasp_traj);
    hand_synergy_pub_mutex_.unlock();
    
    return true;
}

bool ikControl::moveHand(std::string& hand, trajectory_msgs::JointTrajectory& grasp_traj)
{
    hand_synergy_pub_mutex_.lock();
    hand_synergy_pub_.at(hand).publish(grasp_traj);
    hand_synergy_pub_mutex_.unlock();
    
    return true;
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
    map_mutex_.lock();
    group_name = group_map_.at(ee_name);
    if (std::find(chain_names_list_.begin(),chain_names_list_.end(),ee_name) != chain_names_list_.end())
        chain_names.push_back(ee_name);
    else
        chain_names = tree_composition_.at(ee_name);
    map_mutex_.unlock();
    
    // also open the hand(s) we're moving home, but don't wait for it(them)
    std::vector <double > q = {0.0};
    std::vector <double > t = {1.0/hand_max_velocity};
    for(auto& ee:chain_names)
    {
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : opening hand " << ee);
        trajectory_msgs::JointTrajectory grasp_traj;
        moveHand(ee,q,t,grasp_traj);
    }
    
    // if the group is moving, stop it
    this->stop();
    // update planning_init_rs_ with current robot state
    bool target_ok = reset_robot_state(sikm.planning_init_rs_);
    
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
    sikm.end_time_mutex_.lock();
    sikm.movement_end_time_ = ros::Time(0);
    sikm.end_time_mutex_.unlock();
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : \"" << req.attObject.object.id << "\" with \"" << req.ee_name << "\"");
    
    moveit::planning_interface::MoveItErrorCode error_code;
    
    dual_manipulation_shared::ik_response msg;
    msg.seq=req.seq;
    msg.group_name = req.ee_name;
    
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
        msg.data = "error";
        hand_pub.at(local_capability).publish(msg);
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
        map_mutex_.lock();
        std::string group_name(group_map_.at(req.ee_name));
        map_mutex_.unlock();
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
            msg.data = "error";
            hand_pub.at(local_capability).publish(msg);
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
        moveGroups_mutex_.lock();
        error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlan);
        moveGroups_mutex_.unlock();
        if (error_code.val != 1)
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to send trajectory to the controller, returning");
            msg.data = "error";
            hand_pub.at(local_capability).publish(msg);
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
        reset_robot_state(sikm.planning_init_rs_,req.ee_name,trajectory);
        
        // // wait for approach
        bool good_stop = waitForExecution(req.ee_name,trajectory);
        // I didn't make it
        if (!good_stop)
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute approach trajectory, returning");
            msg.data = "error";
            hand_pub.at(local_capability).publish(msg);
            return;
        }
        
        #ifdef SIMPLE_GRASP
        // // moveHand
        std::vector <double > q = {0.4,CLOSED_HAND};
        std::vector <double > t = {0.4/hand_max_velocity,0.5+1.0/hand_max_velocity};
        trajectory_msgs::JointTrajectory grasp_traj;
        moveHand(req.ee_name,q,t,grasp_traj);
        // // wait for hand moved
        good_stop = waitForHandMoved(req.ee_name,q.back(),grasp_traj);
        #else
        // // wait for hand moved
        good_stop = waitForHandMoved(req.ee_name,req.grasp_trajectory.points.back().positions.at(0),req.grasp_trajectory);
        #endif
        // I didn't make it
        if (!good_stop)
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute grasp trajectory, returning");
            msg.data = "error";
            hand_pub.at(local_capability).publish(msg);
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
    
    scene_object_mutex_.lock();
    scene_object_manager_.manage_object(req_obj);
    scene_object_mutex_.unlock();
    
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
    msg.data = "done";
    hand_pub.at(local_capability).publish(msg);
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
    map_mutex_.lock();
    busy.at(capabilities_.type.at(local_capability)).at(req.ee_name) = false;
    map_mutex_.unlock();
    
    return;
}

void ikControl::ungrasp(dual_manipulation_shared::ik_service::Request req)
{
    ik_control_capabilities local_capability = ik_control_capabilities::UNGRASP;
    sikm.end_time_mutex_.lock();
    sikm.movement_end_time_ = ros::Time(0);
    sikm.end_time_mutex_.unlock();
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : \"" << req.attObject.object.id << "\" from \"" << req.ee_name << "\"");
    
    moveit::planning_interface::MoveItErrorCode error_code;
    
    dual_manipulation_shared::ik_response msg;
    msg.seq=req.seq;
    msg.group_name = req.ee_name;
    //NOTE: never check collision for waypoints (at least for now)
    bool check_collisions = false;
    double allowed_distance = 25;
    //ATTENTION: make this more general, depending on the robot
    std::vector<double> single_distances({0.5,0.5,0.5,1.0,1.0,1.0,1.0});
    
    // // get timed trajectory from waypoints
    moveit_msgs::RobotTrajectory trajectory;
    map_mutex_.lock();
    std::string group_name(group_map_.at(req.ee_name));
    map_mutex_.unlock();
    std::lock(sikm.robotState_mutex_,ikCheck_mutex_);
    ik_check_->reset_robot_state(*(sikm.planning_init_rs_));
    sikm.robotState_mutex_.unlock();
    double completed = computeTrajectoryFromWPs(trajectory,req.ee_pose,*ik_check_,group_name,req.ee_name,check_collisions, allowed_distance, single_distances);
    ikCheck_mutex_.unlock();
    
    bool good_stop = false;
    
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
    moveHand(req.ee_name,q,t,grasp_traj);
    // // wait for hand moved
    good_stop = waitForHandMoved(req.ee_name,q.back(),grasp_traj);
    // I didn't make it
    if (!good_stop)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute ungrasp trajectory, returning");
        msg.data = "error";
        hand_pub.at(local_capability).publish(msg);
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
        
        scene_object_mutex_.lock();
        bool ok = scene_object_manager_.manage_object(req_scene);
        scene_object_mutex_.unlock();
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
        moveGroups_mutex_.lock();
        error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlan);
        moveGroups_mutex_.unlock();
        if (error_code.val != 1)
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to send trajectory to the controller, returning");
            msg.data = "error";
            hand_pub.at(local_capability).publish(msg);
            // reset movement_end_time_ in order not to block planning
            sikm.end_time_mutex_.lock();
            sikm.movement_end_time_ = ros::Time::now();
            sikm.end_time_mutex_.unlock();
            return;
        }
        
        // a good, planned trajectory has been successfully sent to the controller
        reset_robot_state(sikm.planning_init_rs_,req.ee_name,trajectory);
        
        // // wait for retreat
        good_stop = waitForExecution(req.ee_name,trajectory);
        // I didn't make it
        if (!good_stop)
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to execute retreat trajectory, returning");
            msg.data = "error";
            hand_pub.at(local_capability).publish(msg);
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
    
    msg.data = "done";
    hand_pub.at(local_capability).publish(msg);
    map_mutex_.lock();
    busy.at(capabilities_.type.at(local_capability)).at(req.ee_name) = false;
    map_mutex_.unlock();
    
    return;
}

bool ikControl::reset_robot_state(const moveit::core::RobotStatePtr& rs)
{
    moveGroups_mutex_.lock();
    moveit::core::RobotState kinematic_state(*(moveGroups_.begin()->second->getCurrentState()));
    moveGroups_mutex_.unlock();
    
    std::unique_lock<std::mutex> lck1(robotState_mutex_,std::defer_lock);
    std::unique_lock<std::mutex> lck2(sikm.robotState_mutex_,std::defer_lock);
    std::lock(lck1,lck2);
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : resetting " << rs->getRobotModel()->getName());
    
    // minimal checks - are more checks needed?
    assert(kinematic_state.getVariableCount() == rs->getVariableCount());
    assert(kinematic_state.getRobotModel()->getName() == rs->getRobotModel()->getName());
    
    for(int i=0; i<rs->getVariableCount(); i++)
        rs->setVariablePosition(i,kinematic_state.getVariablePosition(i));
    
    return true;
}

bool ikControl::reset_robot_state(const moveit::core::RobotStatePtr& rs, std::string ee_name, const moveit_msgs::RobotTrajectory& traj)
{
    std::string group_name;
    map_mutex_.lock();
    group_name = group_map_.at(ee_name);
    map_mutex_.unlock();
    
    std::unique_lock<std::mutex> lck1(robotState_mutex_,std::defer_lock);
    std::unique_lock<std::mutex> lck2(sikm.robotState_mutex_,std::defer_lock);
    std::lock(lck1,lck2);
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : resetting " << rs->getRobotModel()->getName() << " with a trajectory for " << ee_name);
    
    //NOTE: robot_traj, built on robot_model, contains the full robot; trajectory, instead, is only for the group joints
    robot_trajectory::RobotTrajectory robot_traj(rs->getRobotModel(),rs->getJointModelGroup(group_name)->getName());
    robot_traj.setRobotTrajectoryMsg(*rs,traj);
    
    // minimal checks - are more checks needed?
    assert(robot_traj.getLastWayPoint().getVariableCount() == rs->getVariableCount());
    assert(robot_traj.getLastWayPoint().getRobotModel()->getName() == rs->getRobotModel()->getName());
    
    for(int i=0; i<rs->getVariableCount(); i++)
        rs->setVariablePosition(i,robot_traj.getLastWayPoint().getVariablePosition(i));
    
    return true;
}

void ikControl::add_target(const dual_manipulation_shared::ik_service::Request& req)
{
    std::unique_lock<std::mutex>(map_mutex_);
    ik_control_capabilities local_capability = capabilities_.from_name[req.command];
    
    rndmPlan->add_target(req);
    
    busy.at(capabilities_.type.at(local_capability)).at(req.ee_name) = false;
}

bool ikControl::publishTrajectoryPath(const moveit_msgs::RobotTrajectory& trajectory_msg)
{
    std::string group_name;
    map_mutex_.lock();
    group_name = group_map_.at("full_robot");
    map_mutex_.unlock();
    robot_trajectory::RobotTrajectory trajectory(robot_model_,group_name);
    sikm.robotState_mutex_.lock();
    trajectory.setRobotTrajectoryMsg(*(sikm.planning_init_rs_),trajectory_msg);
    sikm.robotState_mutex_.unlock();
    ros::Duration dTs(0.1);
    ros::Duration time(0);
    
    static ros::Publisher joint_state_pub_;
    static bool pub_initialized(false);
    if (!pub_initialized)
    {
        joint_state_pub_ = node.advertise<sensor_msgs::JointState>("ik_control_joint_states",10);
        pub_initialized = true;
    }
    sensor_msgs::JointState js_msg;
    js_msg.name = trajectory_msg.joint_trajectory.joint_names;
    js_msg.header = trajectory_msg.joint_trajectory.header;
    ros::Duration total_time = trajectory_msg.joint_trajectory.points.back().time_from_start;
    
    ros::Rate rate(1.0/dTs.toSec());
    while(time < total_time)
    {
        time += dTs;
        if(time > total_time)
            time = total_time;
        
        if(!trajectory.getStateAtDurationFromStart(time.toSec(),visual_rs_))
        {
            ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to get interpolated state, trajectory.getStateAtDurationFromStart(...) failed");
            return false;
        }
        
        js_msg.header.stamp = ros::Time::now();
        js_msg.position.clear();
        js_msg.velocity.clear();
        for(int i=0; i<js_msg.name.size(); i++)
        {
            js_msg.position.push_back(visual_rs_->getVariablePosition(js_msg.name.at(i)));
            js_msg.velocity.push_back(visual_rs_->getVariableVelocity(js_msg.name.at(i)));
        }
        joint_state_pub_.publish(js_msg);
        ros::spinOnce();
        
        rate.sleep();
    }
    
    return true;
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
    
    rndmPlan.reset(new randomPlanningCapability(sikm));
}

void ikControl::deleteCapabilities()
{
    
}
