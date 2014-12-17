#ifndef TRAJECTORY_PLANNING_ROS_SERVER
#define TRAJECTORY_PLANNING_ROS_SERVER

#include <ros/ros.h>
#include "trajectory_generator.h"
#include "dual_manipulation_shared/ik_service.h"

/**
 * @brief service ik_service:
 *   string ee_name
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

    dual_manipulation::ik_control::trajectory_generator trj_gen;

    bool ik_ros_service(dual_manipulation_shared::ik_service::Request &req, dual_manipulation_shared::ik_service::Response &res);

    ros::ServiceServer service;
};

}
}

#endif // TRAJECTORY_PLANNING_ROS_SERVER