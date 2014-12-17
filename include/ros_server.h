#ifndef TRAJECTORY_PLANNING_ROS_SERVER
#define TRAJECTORY_PLANNING_ROS_SERVER

#include <ros/ros.h>

/**
 * @brief Commands:
 * - 
 * 
 * 
 * 
 */

namespace dual_manipulation
{
namespace ik_control
{

class ros_server
{
public:

private:
    ros::NodeHandle node;
    
    ros::ServiceServer service_server;
};

}
}
#endif // TRAJECTORY_PLANNING_ROS_SERVER