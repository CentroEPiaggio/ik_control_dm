#ifndef TRAJECTORY_PLANNING_ROS_SERVER
#define TRAJECTORY_PLANNING_ROS_SERVER

#include <ros/ros.h>
#include "ik_control.h"
#include "dual_manipulation_shared/ik_service.h"

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

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

//     urdf::Model urdf_model_;
    robot_model::RobotModelPtr kinematic_model_;

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

    /**
     * @brief This function gets the robot kinematic model from ROS
     * 
     * @return void
     */
    void getKinematicModel();
    
    ros::ServiceServer service;
};

}
}

#endif // TRAJECTORY_PLANNING_ROS_SERVER