#ifndef ABSTRACT_CAPABILITY_H_
#define ABSTRACT_CAPABILITY_H_

#include <mutex>

#include <dual_manipulation_shared/ik_control_capabilities.h>
#include <dual_manipulation_shared/ik_service.h>
#include <dual_manipulation_shared/ik_response.h>
#include <dual_manipulation_ik_control/group_structure_manager.h>
// #include <dual_manipulation_ik_control/robot_controller_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <XmlRpcValue.h>

namespace dual_manipulation
{
namespace ik_control
{

// ATTENTION: forward declare... until the dependency from shared_ik_memory is resolved
class RobotControllerInterface;

/**
 * @brief A structure to share resources across implemented capabilities: plan, move, grasp, ...
 */
class shared_ik_memory
{
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
    std::map<std::string,std::string> grasped_obj_map_;
    std::map<std::string,moveit_msgs::AttachedCollisionObject> objects_map_;
    // trajectory execution expected end-time
    std::mutex end_time_mutex_;
    ros::Time movement_end_time_;
    // manage robot group structure
    std::unique_ptr<const GroupStructureManager> groupManager;
    // manage robot controllers
    // TODO: make this const!!!
    std::unique_ptr<RobotControllerInterface> robotController;
};

/**
 * @brief An abstract class implementing a generic capability for use inside ik_control: plan, move, grasp, ...
 */
class abstractCapability
{
public:
    abstractCapability(){}
    virtual ~abstractCapability(){}
    virtual bool isComplete()=0;
    virtual void performRequest(dual_manipulation_shared::ik_serviceRequest req)=0;
    virtual bool getResults(dual_manipulation_shared::ik_response& res)=0;
    virtual bool canRun()=0;
    /// could be associated with a type coming from ik_control_capabilities.h
    virtual bool canPerformCapability(const ik_control_capabilities& ik_capability) const=0;
    virtual void reset(){}
};

}
}

#endif // ABSTRACT_CAPABILITY_H_

// // // #include "lockstrap.h"
// // // 
// // // // class User {
// // // //     class Data {
// // // //         int a;
// // // //         float b;
// // // //         LOCKSTRAP(Data, std::mutex, a,b);
// // // //     } d;
// // // //     std::vector<long> x;
// // // //     // ...
// // // // };