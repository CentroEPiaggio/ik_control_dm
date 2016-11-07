#include "ik_check_capability/ik_check_capability.h"
#include <dual_manipulation_shared/parsing_utils.h>

#include <moveit_msgs/GetPositionIK.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_kdl.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group/capability_names.h>

#include <kdl_parser/kdl_parser.hpp>

#define DEBUG 0
#define CLASS_NAMESPACE "ikCheckCapability::"

using namespace dual_manipulation::ik_control;

ikCheckCapability::ikCheckCapability():robot_model_loader_(new robot_model_loader::RobotModelLoader("robot_description"))
{
    initializeIKCheckCapability(robot_model_loader_->getModel());
}

ikCheckCapability::ikCheckCapability(const moveit::core::RobotModelPtr& kinematic_model)
{
    initializeIKCheckCapability(kinematic_model);
}

void ikCheckCapability::initializeIKCheckCapability(const moveit::core::RobotModelPtr& kinematic_model)
{
    if (!kinematic_model)
    {
        std::cerr<<CLASS_NAMESPACE<<__func__<<" could not find a valid robot_description in ros param server, did you load one?"<<std::endl;
        abort();
    }
    kinematic_model_ = kinematic_model;
    
    setDefaultParameters();
    
    if (node.getParam("ik_control_parameters", ik_control_params))
        parseParameters(ik_control_params);
    
    setParameterDependentVariables();
}

ikCheckCapability::~ikCheckCapability()
{
}

void ikCheckCapability::setDefaultParameters()
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
    
    kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
    kinematic_state_->setToDefaultValues();
    
    // create a local planning scene
    planning_scene_ = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(kinematic_model_));
    empty_planning_scene_ = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(kinematic_model_));
    
    // for the first time, update the planning scene in full
    ros::ServiceClient scene_client = node.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    moveit_msgs::GetPlanningScene srv;
    uint32_t everything = -1; // all bits set to 1, is a binary OR to all possible components
    srv.request.components.components = everything;
    if(!scene_client.call(srv))
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : unable to call " << node.resolveName("get_planning_scene",true) << " service - starting with an empty planning scene...");
    else
    {
        planning_scene_->usePlanningSceneMsg(srv.response.scene);
        ROS_DEBUG_STREAM(CLASS_NAMESPACE << __func__ << " : " << node.resolveName("get_planning_scene",true) << " service returned \n" << srv.response.scene);
    }
    
    // get all possible group names
    group_names_.clear();
    group_names_ = kinematic_model_->getJointModelGroupNames();
    
    scene_sub_ = node.subscribe("move_group/monitored_planning_scene",1,&ikCheckCapability::scene_callback,this);
    
    #if DEBUG>2
    collision_request_.verbose = true;
    #endif
    
    // apart from the first time, when this is done in the constructor after parameters are obtained from the server
    if(is_initialized_)
    {
        setParameterDependentVariables();
    }
    
    gravity = KDL::Vector(0.0,0.0,-9.81);
}

void ikCheckCapability::setParameterDependentVariables()
{
    // initialize simple_chain_ik stuff
    KDL::Tree robot_kdl;
    if (!kdl_parser::treeFromUrdfModel(*(kinematic_model_->getURDF()), robot_kdl))
    {
        ROS_ERROR_STREAM("Failed to construct kdl tree");
        abort();
    }
    tree = std::make_shared<KDL::Tree>(robot_kdl);
    
    tree_fk.reset(new KDL::TreeFkSolverPos_recursive(*tree));
    
#if DEBUG
    std::string robot_root = tree->getRootSegment()->first;
    std::cout << "robot root: " << robot_root << std::endl;
#endif
    
    auto& srdf_groups = kinematic_model_->getSRDF()->getGroups();
    for(auto& group:group_map_)
    {
        if(std::find(group_names_.begin(),group_names_.end(),group.second) == group_names_.end())
            ROS_ERROR_STREAM("Specified group \"" << group.second << "\" (named : " << group.first << ") not present : IK check will not be possible for that group!!!");
        else
        {
            std::string group_real(group.second);
            int group_count = 0;
            moveit::core::JointModelGroup* jmg = kinematic_model_->getJointModelGroup(group_real);
            if(jmg->isChain())
            {
                jmg->setDefaultIKTimeout(default_ik_timeout_);
                jmg->setDefaultIKAttempts(default_ik_attempts_);
                
                // these groups will always be the same, so I just need to find the right index
                while(srdf_groups.at(group_count).name_ != group_real)
                    group_count++;
                const srdf::Model::Group& g(srdf_groups.at(group_count));
                
#if DEBUG
                std::cout << "Looking at group #" << group_count << std::endl;
                std::cout << "group_real =" << group_real << " | g.name_=" << g.name_ << std::endl;
                std::cout << "g.chains_.size()=" << g.chains_.size();
                if(g.chains_.size() > 0)
                    std::cout << " | g.chains_.at(0)=" << g.chains_.at(0).first << ">" << g.chains_.at(0).second;
                std::cout << std::endl;
                std::cout << "chain root: " << g.chains_.at(0).first << std::endl;
                std::cout << "chain ee: " << g.chains_.at(0).second << std::endl;
#endif
                std::string end_effector = g.chains_.at(0).second;
                std::string root = g.chains_.at(0).first;
                solvers[group.first].reset(new ChainAndSolvers(tree,tree_fk,root,end_effector,gravity));
                initialize_solvers(*solvers[group.first]);
            }
        }
    }
    
    is_initialized_ = true;
}

void ikCheckCapability::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    parseSingleParameter(params,chain_names_list_,"chain_group_names",1);
    parseSingleParameter(params,tree_names_list_,"tree_group_names",1);
    parseSingleParameter(params,kinematics_only_,"kinematics_only");
    
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
            ROS_WARN_STREAM("No composition is specified for tree '" << tree << "': check the yaml configuration.");
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
        
        parseSingleParameter(params,default_ik_timeout_,"default_ik_timeout");
    int tmp_int_param = (int)default_ik_attempts_;
    parseSingleParameter(params,tmp_int_param,"default_ik_attempts");
    if(tmp_int_param >= 0)
        default_ik_attempts_ = (unsigned int)tmp_int_param;
    else
        ROS_WARN_STREAM("Attempted to set default_ik_attempts to a negative value; using default instead.");
}

bool ikCheckCapability::find_group_ik(std::string group_name, const geometry_msgs::Pose& ee_pose, std::vector< double >& solution, const std::vector< double >& initial_guess, bool check_collisions, bool return_approximate_solution, unsigned int attempts, double timeout, const std::map< std::string, std::string >& allowed_collisions)
{
    std::unique_lock<std::mutex> ul(interface_mutex_);
    
    if(!can_be_managed(group_name))
        return false;

    // get default allowed collision matrix and add user-specified entries (this always needs to be done just once)
    scene_mutex_.lock();
    acm_.clear();
    acm_ = planning_scene_->getAllowedCollisionMatrix();
    for(auto& ac:allowed_collisions)
        acm_.setEntry(ac.first,ac.second,true);
    scene_mutex_.unlock();
    
    // call private implementation
    return find_ik(group_name, ee_pose, solution, initial_guess, check_collisions, return_approximate_solution, attempts, timeout);
}

bool ikCheckCapability::find_closest_group_ik(std::string group_name, const geometry_msgs::Pose& ee_pose, std::vector< double >& solution, double allowed_distance, std::vector< double > single_distances, unsigned int trials_nr, const std::vector< double >& initial_guess, bool check_collisions, bool return_approximate_solution, unsigned int attempts, double timeout, const std::map< std::string, std::string >& allowed_collisions)
{
    std::unique_lock<std::mutex> ul(interface_mutex_);
    
    // manage interface errors
    if(trials_nr == 0)
    {
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : asked to find closest IK out of ZERO trials - returning");
        return false;
    }
    if(!can_be_managed(group_name))
        return false;
    
    // prepare arguments for the private implementation
    
    const moveit::core::JointModelGroup* jmg = kinematic_model_->getJointModelGroup(group_map_.at(group_name));
    // get default allowed collision matrix and add user-specified entries (this always needs to be done just once)
    scene_mutex_.lock();
    acm_.clear();
    acm_ = planning_scene_->getAllowedCollisionMatrix();
    for(auto& ac:allowed_collisions)
        acm_.setEntry(ac.first,ac.second,true);
    scene_mutex_.unlock();
    
    // best solution found so far and its distance from the starting state
    std::vector<double> best_found;
    double best_distance = -1.0;
    
    // variables used at each cycle
    double distance;
    std::vector<double> curr_position;
    std::vector<double> internal_initial_guess(initial_guess);
    std::vector<double> ref_position;
    kinematic_state_->copyJointGroupPositions(jmg,ref_position);
    if(!initial_guess.empty() && (initial_guess.size() == jmg->getVariableCount()))
    {
        ref_position.clear();
        ref_position = initial_guess;
    }
    for(int i=0; i<trials_nr; i++)
    {
        // change the initial guess at every trial
        if(i!=0)
        {
            // NOTE: there is no need to reset as the initial_guess will be used internally anyway
            kinematic_state_->setToRandomPositions(jmg);
            internal_initial_guess.clear();
            kinematic_state_->copyJointGroupPositions(jmg,internal_initial_guess);
        }
        
        // call private implementation
        if(!find_ik(group_name, ee_pose, solution, internal_initial_guess, check_collisions, return_approximate_solution, attempts, timeout))
            continue;
        else
            ROS_DEBUG_STREAM("I found a solution with find_group_ik_impl!");
        
        // a solution has been found: compute the distance from initial_guess
        kinematic_state_->copyJointGroupPositions(jmg,curr_position);
        distance = 0;
        std::vector<double> single_distances_local;
        for(int j=0; j<curr_position.size(); j++)
        {
            distance += std::abs(curr_position.at(j) - ref_position.at(j));
            single_distances_local.push_back(std::abs(curr_position.at(j) - ref_position.at(j)));
        }
        
        bool high_joint_distance = false;
        std::string single_distance_str;
        for(int j=0; j<single_distances.size() && j<single_distances_local.size(); j++)
        {
            single_distance_str = single_distance_str + ", " + std::to_string(single_distances_local.at(j));
            if(single_distances_local.at(j) > single_distances.at(j))
            {
                high_joint_distance = true;
            }
        }
        if(high_joint_distance)
        {
            #if DEBUG>0
            ROS_WARN_STREAM("Trial #" << i << ": distance = " << distance << " | single_distances are " << (high_joint_distance?"":"NOT ") << "high = " << single_distance_str);
            #endif
            continue;
        }
        
        ROS_INFO_STREAM("Trial #" << i << ": distance = " << distance << " | single_distances are " << (high_joint_distance?"":"NOT ") << "high = " << single_distance_str);
        
        // in case I'm closer (or it's the first time I found a solution), update best found so far
        if(best_distance < 0 || distance < best_distance)
        {
            best_distance = distance;
            best_found.swap(solution);
            #if DEBUG>0
            std::cout << "it #" << i << " out of " << trials_nr << std::endl;
            for(int j=0; j<curr_position.size(); j++)
                std::cout << "q(" << j << "): " << curr_position.at(j) << " | " << ref_position.at(j) << std::endl;
            std::cout << std::endl;
            #endif
        }
        
        // if I found a solution respecting the threshold, return
        if(best_distance < allowed_distance)
        {
            solution.swap(best_found);
            return true;
        }
    }
    
    // solutions is always last one, while best_found keeps the best so far: swap at the end
    solution.swap(best_found);
    
    // didn't find any solution respecting the threshold: return false
    return false;
}

void ikCheckCapability::scene_callback(const moveit_msgs::PlanningScene::ConstPtr& plan_msg)
{
    #if DEBUG>1
    if(kinematics_only_)
        ROS_INFO_STREAM("ikCheckCapability::scene_callback : updating planning scene");
    #endif
    
    // update the internal planning scene, considering whether or not is_diff flag is set to true
    scene_mutex_.lock();
    planning_scene_->usePlanningSceneMsg(*plan_msg);
    scene_mutex_.unlock();
    
    // ROS_INFO_STREAM("ikCheckCapability::scene_callback - plan_msg:\n" << *plan_msg << std::endl);
}

//NOTE: this is a joint model group we could use (all possible groups exist this way, e.g. even head in vito)
// moveit::core::JointModelGroup* bim_group = kinematic_model_->getJointModelGroup("dual_hand_arm");
//NOTE: the following names are from the SRDF file, not the link names
// const std::vector <std::string > ee_names = bim_group->getAttachedEndEffectorNames();
//NOTE: getEndEffectorTips() works if the jm_group is a tree, returning the link names of all the end-effectors
//      BUT does not work for chains (i.e.: works for bimanual group, but doesn't for single hand/arm groups)
// std::vector <std::string > tips;
// bim_group->getEndEffectorTips(tips);
//NOTE: this returns {left_arm | left_hand_arm | right_arm | right_hand_arm}
// const std::vector <std::string >& subgroups = bim_group->getSubgroupNames();
//NOTE: isChain() and isEndEffector() always return false for trees
//NOTE: kinematic_model_->getRootLinkName() could be used instead of "world", just to be sure

bool ikCheckCapability::find_ik(std::string ee_name, const geometry_msgs::Pose& ee_pose, std::vector< double >& solution, const std::vector< double >& initial_guess, bool check_collisions, bool return_approximate_solution, unsigned int attempts, double timeout)
{
    if(!kinematic_model_->hasEndEffector(ee_name))
    {
        ROS_ERROR_STREAM("End-effector " << ee_name << " not found : returning!");
        return false;
    }
    
    // construct necessary inputs
    const moveit::core::JointModelGroup* jmg = kinematic_model_->getEndEffector(ee_name);
    
    // // NOTE: this should be alredy done inside setFromIK anyway...
    // if (attempts == 0)
    //   attempts = default_ik_attempts_;
    // if (timeout == 0.0)
    //   timeout = default_ik_timeout_;
    
    moveit::core::GroupStateValidityCallbackFn constraint;
    if (check_collisions)
    {
        constraint = boost::bind(&ikCheckCapability::is_collision_free, this,_1,_2,_3);
    }
    else
    {
        constraint = boost::bind(&ikCheckCapability::is_self_collision_free, this,_1,_2,_3);
    }
    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = return_approximate_solution;
    
    if(!initial_guess.empty())
        if(initial_guess.size() == jmg->getVariableCount())
            kinematic_state_->setJointGroupPositions(jmg,initial_guess);
        else
            ROS_WARN_STREAM("Initial guess passed as parameter has a wrong dimension : using default position instead");
        
        if(!kinematic_state_->setFromIK(jmg,ee_pose,attempts,timeout,constraint,options))
            return false;
    
    kinematic_state_->copyJointGroupPositions(jmg,solution);
    return true;
}

bool ikCheckCapability::is_collision_free(moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup *jmg, const double* q)
{
    ROS_DEBUG_STREAM("ikCheckCapability::is_collision_free has been called for group " << jmg->getName());
    
    std::unique_lock<std::mutex> ul(scene_mutex_);
    
    std::vector<double> initial_position;
    robot_state->copyJointGroupPositions(jmg,initial_position);
    
    collision_result_.clear();
    robot_state->setJointGroupPositions(jmg,q);
    planning_scene_->checkCollision(collision_request_, collision_result_,*robot_state,acm_);
    
    robot_state->setJointGroupPositions(jmg,initial_position);
    //   std::cout << "Found " << collision_result_.contact_count << " contact(s) (up to a max of " << collision_request_.max_contacts << "):\n";
    //   for(auto& coll:collision_result_.contacts)
    //     std::cout << coll.first.first << " <> " << coll.first.second << " = " << coll.second.size() << std::endl;
    
    return (!collision_result_.collision);
}

bool ikCheckCapability::is_self_collision_free(moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup *jmg, const double* q)
{
    ROS_DEBUG_STREAM("ikCheckCapability::is_self_collision_free has been called for group " << jmg->getName());
    
    std::vector<double> initial_position;
    robot_state->copyJointGroupPositions(jmg,initial_position);
    
    collision_result_.clear();
    robot_state->setJointGroupPositions(jmg,q);
    empty_planning_scene_->checkCollision(collision_request_, collision_result_,*robot_state,acm_);
    
    robot_state->setJointGroupPositions(jmg,initial_position);
    //   std::cout << "Found " << collision_result_.contact_count << " contact(s) (up to a max of " << collision_request_.max_contacts << "):\n";
    //   for(auto& coll:collision_result_.contacts)
    //     std::cout << coll.first.first << " <> " << coll.first.second << " = " << coll.second.size() << std::endl;
    
    return (!collision_result_.collision);
}

bool ikCheckCapability::reset_robot_state(std::string group, std::string named_target)
{
    std::unique_lock<std::mutex> ul(interface_mutex_);
    
    if(group.empty())
    {
        kinematic_state_->setToDefaultValues();
        return true;
    }
    
    if(group_map_.count(group) == 0)
    {
        ROS_ERROR_STREAM("ikCheckCapability::reset_robot_state : " << group << " is not a known group - returning");
        return false;
    }
    
    const moveit::core::JointModelGroup* jmg = kinematic_model_->getJointModelGroup(group_map_.at(group));
    return kinematic_state_->setToDefaultValues(jmg,named_target);
}

bool ikCheckCapability::reset_robot_state(const moveit::core::RobotState& rs)
{
    std::unique_lock<std::mutex> ul(interface_mutex_);
    
    // minimal checks - are more checks needed?
    assert(kinematic_state_->getVariableCount() == rs.getVariableCount());
    assert(kinematic_state_->getRobotModel()->getName() == rs.getRobotModel()->getName());
    
    for(int i=0; i<rs.getVariableCount(); i++)
        kinematic_state_->setVariablePosition(i,rs.getVariablePosition(i));
    
    return true;
}

bool ikCheckCapability::reset_robot_state(std::string group, std::vector<double> target)
{
    std::unique_lock<std::mutex> ul(interface_mutex_);
    
    if(group_map_.count(group) == 0)
    {
        ROS_ERROR_STREAM("ikCheckCapability::reset_robot_state : " << group << " is not a known group - returning");
        return false;
    }
    
    const moveit::core::JointModelGroup* jmg = kinematic_model_->getJointModelGroup(group_map_.at(group));
    if(jmg->getVariableCount() != target.size())
    {
        ROS_ERROR_STREAM("ikCheckCapability::reset_robot_state : dimension mismatch - " << group << " has " << jmg->getVariableCount() << " joints, target has " << target.size() << " values - returning");
        return false;
    }
    
    kinematic_state_->setJointGroupPositions(jmg,target);
    return true;
}

moveit::core::RobotState ikCheckCapability::get_robot_state()
{
    std::unique_lock<std::mutex> ul(interface_mutex_);
    
    return *kinematic_state_;
}

bool ikCheckCapability::is_state_collision_free(moveit::core::RobotState* robot_state, std::string group, bool self_collision_only)
{
    std::unique_lock<std::mutex> ul(interface_mutex_);
    
    if(group_map_.count(group) == 0)
    {
        ROS_ERROR_STREAM("ikCheckCapability::reset_robot_state : " << group << " is not a known group - returning");
        return false;
    }
    
    const moveit::core::JointModelGroup* jmg = kinematic_model_->getJointModelGroup(group_map_.at(group));
    double q[jmg->getVariableCount()];
    robot_state->copyJointGroupPositions(jmg,q);
    
    if(self_collision_only)
        return is_self_collision_free(robot_state,jmg,q);
    else
        return is_collision_free(robot_state,jmg,q);
}

planning_scene::PlanningSceneConstPtr ikCheckCapability::get_planning_scene(bool updated)
{
    std::unique_lock<std::mutex> ul(interface_mutex_);
    
    if(updated)
        return planning_scene_;
    else
        return empty_planning_scene_;
}

bool ikCheckCapability::can_be_managed(const std::string& group_name)
{
    // manage interface errors
    if(group_map_.count(group_name) == 0)
    {
        ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : " << group_name << " is not a known group - returning");
        return false;
    }
    if(std::find(tree_names_list_.begin(),tree_names_list_.end(),group_name) != tree_names_list_.end())
    {
        ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : " << group_name << " is a tree, but this function no longer supports trees - returning");
        return false;
    }
    return true;
}

void ikCheckCapability::initialize_solvers(ChainAndSolvers& container) const
{
    KDL::JntArray q_min, q_max;
    q_min.resize(container.jointNames().size());
    q_max.resize(container.jointNames().size());
    int j=0;
    for (auto& joint_name:container.jointNames())
    {
        if(kinematic_model_->getURDF()->getJoint(joint_name)->safety)
        {
            q_max(j)=kinematic_model_->getURDF()->getJoint(joint_name)->safety->soft_upper_limit;
            q_min(j)=kinematic_model_->getURDF()->getJoint(joint_name)->safety->soft_lower_limit;
        }
        else
        {
            q_max(j)=kinematic_model_->getURDF()->getJoint(joint_name)->limits->upper;
            q_min(j)=kinematic_model_->getURDF()->getJoint(joint_name)->limits->lower;
        }
        j++;
    }
    
    // use default values for max_iter and eps, they will be changed at each IK request
    int max_iter(20);
    double eps(5e-4);
    
    if(!container.setSolverParameters(q_min,q_max,max_iter,eps,150,1e-5,1e-5) || !container.initSolvers())
    {
        std::cout << CLASS_NAMESPACE << __func__ << " : unable to initialize the solvers! Returning..." << std::endl;
        abort();
    }
}

std::unique_ptr< ChainAndSolvers >& ikCheckCapability::getChainAndSolvers(const std::string& group_name)
{
    if(solvers.count(group_name))
        return solvers[group_name];
    else
        return empty_ptr;
}
