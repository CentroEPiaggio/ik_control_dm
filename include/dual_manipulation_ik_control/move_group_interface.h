#include <ros/common.h>

#if ROS_VERSION_MINIMUM(1,12,6)
    #include <moveit/move_group_interface/move_group_interface.h>

    namespace dual_manipulation
    {
    namespace ik_control
    {
        /// Motion Planning error code
        typedef moveit::planning_interface::MoveItErrorCode MPErrorCode;
        
        /// Motion plan
        typedef moveit::planning_interface::MoveGroupInterface::Plan MotionPlan;
        
        /// Move Group
        typedef moveit::planning_interface::MoveGroupInterface MoveGroup;
    }
    }

#else

    #include <moveit/move_group_interface/move_group.h>

    namespace dual_manipulation
    {
    namespace ik_control
    {
        /// Motion Planning error code
        typedef moveit::planning_interface::MoveItErrorCode MPErrorCode;
        
        /// Motion plan
        typedef moveit::planning_interface::MoveGroup::Plan MotionPlan;
        
        /// Move Group
        typedef moveit::planning_interface::MoveGroup MoveGroup;
    }
    }

#endif
