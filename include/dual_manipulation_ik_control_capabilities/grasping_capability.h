#ifndef GRASPING_CAPABILITY_H_
#define GRASPING_CAPABILITY_H_

#include <dual_manipulation_ik_control_capabilities/abstract_capability.h>
#include <atomic>
#include <moveit/robot_model_loader/robot_model_loader.h>

namespace dual_manipulation
{
namespace ik_control
{

class GraspingCapability : public abstractCapability
{
public:
    GraspingCapability(shared_ik_memory& sikm_, const ros::NodeHandle& node_ = ros::NodeHandle());
    virtual ~GraspingCapability();
    virtual bool isComplete();
    virtual void performRequest(dual_manipulation_shared::ik_serviceRequest req);
    virtual bool getResults(dual_manipulation_shared::ik_response& res);
    virtual bool canRun();
    virtual bool canPerformCapability(const ik_control_capabilities& ik_capability) const;
    virtual void reset();
    
private:
    shared_ik_memory& sikm;
    bool kinematics_only_;
    double hand_max_velocity;   // maximum hand velocity : avg is 2.0, closes completely [0.0->1.0] in half a second
    std::map<std::string,std::vector<std::string>> allowed_collision_prefixes_;
    std::map<std::string,std::vector<std::string>> allowed_collisions_;
    std::map<std::string,std::string> hand_actuated_joint_;
    std::mutex map_mutex_; // hand_actuated_joint_
    const ik_control_capability capabilities_;
    std::string robot_description_;
    
    // ros variables
    ros::NodeHandle node;

    // TOPIC FOR PUBLISHING WAYPOINTS IK TESTING (MIRKO)
    ros::Publisher waypoints_pub = node.advertise<geometry_msgs::Pose>("waypoints_topic", 1000);
    // remove above after testing

    // TODO: remove this in favor of a shared planning_scene_monitor
    ros::ServiceClient scene_client_;
    
    // interface and results variables
    std::atomic_bool busy;
    dual_manipulation_shared::ik_response response_;
    
    // MoveIt! variables
    moveit::core::RobotModelPtr robot_model_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    
private:

    /**
     * @brief utility function for publishing waypoints on a topic for IK testing (Mirko)
     * 
     * @param waypoints a vector of geometry_msgs::Pose 
     * 
     * @return void
     */
    void publishWaypointsTopic(std::vector <geometry_msgs::Pose > waypoints);

    /**
     * @brief Utility function to parse parameters
     * 
     * @param params params got from the parameter server
     */
    void parseParameters(XmlRpc::XmlRpcValue& params);
    
    /**
     * @brief Utility function to set class variables which depend on parameters
     */
    void setParameterDependentVariables();
    
    /**
     * @brief handler function for grasping an object
     * 
     * @param req the same req from the @e ik_service
     */
    void grasp(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief handler function for ungrasping an object
     * 
     * @param req the same req from the @e ik_service
     */
    void ungrasp(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief Read grasp details from the database / serialized grasp file
     * 
     * @return false in case of failure, true otherwise
     */
    bool readGraspFromDatabase(dual_manipulation_shared::ik_service::Request& req);
    
    /**
     * @brief Change frame of reference of the grasp trajectory to the current object frame
     * 
     * @param object_pose the current pose of the object (to be used for the local grasp trajectory @p ee_pose)
     * @param ee_pose a grasp trajectory expressed in object_frame, which will be returned expressed in world frame (i.e., each frame will be "pre-multiplied" by object_pose)
     */
    void changeFrameToPoseVector(geometry_msgs::Pose object_pose_msg, std::vector<geometry_msgs::Pose>& ee_pose);
};

}
}

#endif // GRASPING_CAPABILITY_H_
