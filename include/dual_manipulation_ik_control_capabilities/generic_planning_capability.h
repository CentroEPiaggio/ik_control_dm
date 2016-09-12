#ifndef GENERIC_PLANNING_CAPABILITY_H_
#define GENERIC_PLANNING_CAPABILITY_H_

#include <dual_manipulation_ik_control_capabilities/abstract_capability.h>

/**
 * @brief Here define stuff which is needed from various possible planning capabilities, that will be specialized later on
 */

namespace dual_manipulation
{
namespace ik_control
{

enum class ik_target_type
{
    POSE_TARGET,
    JOINT_TARGET,
    NAMED_TARGET
};

struct ik_target
{
    ik_target(){};
    ik_target(std::vector<geometry_msgs::Pose> ee_poses,std::string ee_name):ee_poses(ee_poses),ee_name(ee_name),type(ik_target_type::POSE_TARGET){};
    ik_target(geometry_msgs::Pose ee_pose,std::string ee_name):ee_poses({ee_pose}),ee_name(ee_name),type(ik_target_type::POSE_TARGET){};
    ik_target(std::string target_name,std::string ee_name):target_name(target_name),ee_name(ee_name),type(ik_target_type::NAMED_TARGET){};
    ik_target(std::vector<std::vector<double>> joints,std::string ee_name):joints(joints),ee_name(ee_name),type(ik_target_type::JOINT_TARGET){};
    ik_target(std::vector<double> joint,std::string ee_name):joints({joint}),ee_name(ee_name),type(ik_target_type::JOINT_TARGET){};
    
    std::vector<geometry_msgs::Pose> ee_poses;
    std::string ee_name;
    std::vector<std::vector<double>> joints;
    std::string target_name;
    ik_target_type type;
};

class GenericPlanningCapability : public abstractCapability
{
    
};

}
}

#endif // GENERIC_PLANNING_CAPABILITY_H_
