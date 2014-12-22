#ifndef TRAJECTORY_PLANNING_ROS_SERVER
#define TRAJECTORY_PLANNING_ROS_SERVER

#include <ros/ros.h>
#include "ik_control.h"
#include "dual_manipulation_shared/ik_service.h"

/**
 * @brief service ik_service:
 *   string ee_name
 *   double time
 *   geometry_msgs/Pose ee_pose
 *   ---
 *   bool ack
 */

namespace dual_manipulation
{
namespace ik_control
{

class ros_server
{
public:
    ros_server();
    ~ros_server();
private:
    ros::NodeHandle node;

    ros::ServiceServer service_server;

    ikControl IKControl;

    bool ik_ros_service(dual_manipulation_shared::ik_service::Request &req, dual_manipulation_shared::ik_service::Response &res);

    ros::ServiceServer service;
};

}
}

#endif // TRAJECTORY_PLANNING_ROS_SERVER