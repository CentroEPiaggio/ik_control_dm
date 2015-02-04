#ifndef TRAJECTORY_PLANNING_ROS_SERVER
#define TRAJECTORY_PLANNING_ROS_SERVER

#include <ros/ros.h>
#include "ik_control.h"
#include "dual_manipulation_shared/ik_service.h"

namespace dual_manipulation
{
namespace ik_control
{

/**
  * @brief This class provides a service (@e ik_service) by which the user can move the end effectors
  * 
  */
class ros_server
{
public:
    ros_server();
    ~ros_server();
private:
    ros::NodeHandle node;

    ros::ServiceServer service_server;

    ikControl IKControl_;

    /**
     * @brief This is the @e ik_service callback, it calls a public method of the inner class
     * 
     * @param req 
     *   string ee_name
     *   double time
     *   geometry_msgs/Pose ee_pose
     * @param res
     *   bool ack
     * @return bool
     */
    bool ik_ros_service(dual_manipulation_shared::ik_service::Request &req, dual_manipulation_shared::ik_service::Response &res);
    
    ros::ServiceServer service;
};

}
}

#endif // TRAJECTORY_PLANNING_ROS_SERVER