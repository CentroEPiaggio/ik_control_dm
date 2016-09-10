#include "shared_ik_memory.h"
#include <dual_manipulation_shared/parsing_utils.h>

using namespace dual_manipulation::ik_control;

shared_ik_memory::shared_ik_memory(XmlRpc::XmlRpcValue& params, ros::NodeHandle& nh)
{
    ik_control_params = &params;
    movement_end_time_ = ros::Time::now();
    groupManager.reset(new GroupStructureManager(*ik_control_params));
    sceneObjectManager.reset(new SceneObjectManager(*ik_control_params,*groupManager));
    
    /// important condition: full_robot group has to exist (there are some assumptions around the code)
    bool full_robot_exists = groupManager->getGroupInSRDF("full_robot",full_robot_group_);
    assert(full_robot_exists);
    
    parseParameters(params);
    
    for(auto group_name:groupManager->get_group_map())
    {
        movePlans_[group_name.first];
    }
    robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader(robot_description_));
    robot_model_ = robot_model_loader_->getModel();
    
    planning_init_rs_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_));
    
    robotStateManager.reset(new RobotStateManager(robot_model_,joint_states_,full_robot_group_));
    robotController.reset(new RobotControllerInterface(*ik_control_params,*groupManager,*robotStateManager,nh));
    
    reset();
}

void shared_ik_memory::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    // mandatory parameters
    bool mandatory_parameters(true);
    mandatory_parameters = mandatory_parameters & parseSingleParameter(params,joint_states_,"joint_states");
    mandatory_parameters = mandatory_parameters & parseSingleParameter(params,robot_description_,"robot_description");
    assert(mandatory_parameters);
}

void shared_ik_memory::reset()
{
    robotStateManager->reset_robot_state(planning_init_rs_,full_robot_group_,robotState_mutex_);
    movePlans_mutex_.lock();
    for(auto& plan:movePlans_){ move_group_interface::MoveGroup::Plan tmp_plan; std::swap(plan.second,tmp_plan);}
    movePlans_mutex_.unlock();
    end_time_mutex_.lock();
    movement_end_time_ = ros::Time::now();
    end_time_mutex_.unlock();
}
