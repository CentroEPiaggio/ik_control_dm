#ifndef SHARED_IK_MEMORY_H_
#define SHARED_IK_MEMORY_H_

#include <mutex>
#include <dual_manipulation_ik_control/group_structure_manager.h>
#include <dual_manipulation_ik_control/robot_controller_interface.h>
#include <dual_manipulation_ik_control/robot_state_manager.h>
#include "scene_object_manager.h"
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <XmlRpcValue.h>

namespace dual_manipulation
{
namespace ik_control
{

/**
 * @brief A structure to share resources across implemented capabilities: plan, move, grasp, ...
 */
class shared_ik_memory
{
public:
    shared_ik_memory();
    ~shared_ik_memory() {}
public:
    std::mutex m;
    XmlRpc::XmlRpcValue* ik_control_params;
    // share the robot state to use for next planning
    std::mutex robotState_mutex_;
    moveit::core::RobotStatePtr planning_init_rs_;
    // share the planning scene among planning capabilities
    std::mutex planningScene_mutex_;
    planning_scene::PlanningSceneConstPtr planning_scene_;
    // share the motion plans among planning/control capabilities
    std::mutex movePlans_mutex_;
    std::map<std::string,moveit::planning_interface::MoveGroup::Plan> movePlans_;
    // share objects which have to be in the scene
    std::mutex map_mutex_;
    // trajectory execution expected end-time
    std::mutex end_time_mutex_;
    ros::Time movement_end_time_;
    // manage robot group structure
    std::unique_ptr<const GroupStructureManager> groupManager;
    // manage robot controllers
    std::unique_ptr<const RobotControllerInterface> robotController;
    // manage robot states
    std::unique_ptr<const RobotStateManager> robotStateManager;
    // managing the objects in the scene
    std::unique_ptr<SceneObjectManager> sceneObjectManager;
};

}
}

#endif // SHARED_IK_MEMORY_H_
