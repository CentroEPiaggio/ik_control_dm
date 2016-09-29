#ifndef IK_CONTROL_H
#define IK_CONTROL_H

#include <dual_manipulation_shared/ik_service.h>
#include <dual_manipulation_shared/scene_object_service.h>
#include "ik_check_capability/ik_check_capability.h"
#include <thread>
#include <XmlRpcValue.h>
#include <mutex>
#include <std_msgs/String.h>

// capabilities definition
#include <dual_manipulation_shared/ik_control_capabilities.h>
#include <dual_manipulation_ik_control_capabilities/random_planning_capability.h>
#include <dual_manipulation_ik_control_capabilities/trajectory_execution_capability.h>
#include <dual_manipulation_ik_control_capabilities/grasping_capability.h>
#include <dual_manipulation_ik_control_capabilities/sliding_capability.h>

namespace dual_manipulation
{
namespace ik_control
{

/**
 * @brief Class to manage a single object in an atomic manner, making sure that it gets assigned a desired value (when this object goes out of scope, it makes the assignment using the appropriate LOCK-able variable)
 * 
 * @p LOCK has to be lockable, @p T has to provide a copy constructor and the assignment operator (operator=)
 */
template<typename LOCK, typename T>
class ObjectLocker
{
    const LOCK& m;
    T& flag;
    const T result;
public:
    ObjectLocker(const LOCK& m_,T& flag_,const T& result_) : m(m_), flag(flag_), result(result_) {}
    ~ObjectLocker() { std::unique_lock<LOCK>(m); flag = result;}
};
  
/**
  * @brief This is a class that is used from the ros_server to perform a desired ik using a dedicated thread (one for each end effector).
  * 
  */
class ikControl
{
public:
    ikControl();
    ~ikControl() {}
    
    /**
     * @brief interface function to perform the @e ik_service
     * 
     * @param req
     *   the req from the @e ik_service
     * @return bool
     */
    bool perform_ik(dual_manipulation_shared::ik_service::Request &req);
    
    /**
     * @brief interface function to manage objects
     * 
     * @param req
     *   the req from the @e scene_object_service
     * @return bool
     */
    bool manage_object(dual_manipulation_shared::scene_object_service::Request &req);
    
private:
    // manage IK requests
    std::unique_ptr<ikCheckCapability> ik_check_legacy_;
    
    // MoveIt! variables
    moveit::core::RobotModelPtr robot_model_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    
    // ros variables
    ros::NodeHandle node;
    std::map<ik_control_capabilities,ros::Publisher> hand_pub;
    
    // utility variables
    std::vector<std::unique_ptr<std::thread>> used_threads_;
    std::map<ik_control_capability_types,std::map<std::string,bool>> busy;
    const ik_control_capability capabilities_;
    std::mutex map_mutex_; // busy
    std::mutex ikCheck_mutex_;
    std::string joint_states_;
    std::string robot_description_;
    std::string full_robot_group_;
    
    // managing external parameters
    XmlRpc::XmlRpcValue ik_control_params;
    
    double hand_max_velocity;   // maximum hand velocity : avg is 2.0, closes completely [0.0->1.0] in half a second
    double epsilon_;            // IK tolerance used by KDLKinematicsPlugin
    
    // shared parameters between capabilities
    std::unique_ptr<shared_ik_memory> sikm;
    // capabilities
    std::unique_ptr<randomPlanningCapability> rndmPlan;
    std::unique_ptr<TrajectoryExecutionCapability> trajExecute;
    std::unique_ptr<GraspingCapability> graspPlanExecute;
    std::unique_ptr<SlidingCapability> slidePlan;
    
    /**
     * @brief utility function to parse parameters from the parameter server
     * 
     * @param params
     *   all useful params got from the parameter server
     * @return void
     */
    void parseParameters(XmlRpc::XmlRpcValue& params);
    
    /**
     * @brief utility function to set all class parameters to their default value
     * 
     * @return void
     */
    void setDefaultParameters();
    
    /**
     * @brief utility function to set class variables which depend on parameters
     * 
     * @return void
     */
    void setParameterDependentVariables();
    
    /**
     * @brief this is the thread body to perform IK feasibility check (no collision considered)
     * 
     * @param req
     *   the same req from the @e ik_service
     * @return void
     */
    void ik_check_thread(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief this is the thread body to perform trajectory generation
     * 
     * @param req
     *   the same req from the @e ik_service
     * 
     * @return void
     */
    void planning_thread(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief execute last planned path
     * 
     * @return void
     */
    void execute_plan(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief function to move a group to its home position and to open the hand
     * 
     * @param ee_name
     *   which end-effector bring back home
     * @return void
     */
    void simple_homing(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief handler function for grasping an object
     * 
     * @param req
     *   the same req from the @e ik_service
     * @return void
     */
    void grasp(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief handler function for ungrasping an object
     * 
     * @param req
     *   the same req from the @e ik_service
     * @return void
     */
    void ungrasp(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief handler function for planning a slide motion
     * 
     * @param req
     *   the same req from the @e ik_service
     * @return void
     */
    void plan_slide(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief clear all current busy flags
     * 
     */
    inline void free_all(){ map_mutex_.lock(); for(auto& item:busy) for(auto& item2:item.second) item2.second = false; map_mutex_.unlock(); reset();}
    
    /**
     * @brief resets all robot states and movePlans
     */
    void reset();
    
    /**
     * @brief function to check whether a capability is busy, and to lock it in case it is
     * 
     * @param ee_name
     *    end-effector name
     * @param capability
     *    capability to check
     */
    bool is_free_make_busy(std::string ee_name, std::string capability);
    
    /**
     * @brief add a target to the internal targets list
     * 
     * @param req the same req from the @e ik_service
     */
    void add_target(const dual_manipulation_shared::ik_service::Request& req);
    
    /**
     * @brief create instances of the various capabilities which will be used inside ik_control
     */
    void instantiateCapabilities();
};

}
}

#endif // IK_CONTROL_H
