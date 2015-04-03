#ifndef IKCHECKCAPABILITY_H
#define IKCHECKCAPABILITY_H

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <mutex>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <dual_manipulation_shared/ik_service.h>

namespace dual_manipulation
{
namespace ik_control
{
  
/**
  * @brief This is a class to manage inverse kinematics checking with different implementations
  * 
  */
class ikCheckCapability
{
public:
    ikCheckCapability();
    ikCheckCapability(const moveit::core::RobotModelPtr& kinematic_model);
    ~ikCheckCapability();
    
    /**
     * @brief interface function to manage IK requests
     * 
     * @param req
     *   the req from the @e ik_ros_service
     * @return bool
     */
    bool manage_ik(dual_manipulation_shared::ik_service::Request req);
  
    /**
     * @brief interface function to call IK internal solver; this should present the same interface independently from the fact that the group is a chain or a tree
     * 
     * @param group_name name of the group we want to find IK for
     * @param ee_poses desired poses for all the end-effectors of the group
     * @param solutions the solutions found
     * @param initial_guess the starting point for the IK search (must be for the whole group)
     *        @default empty vector, using the default robot configuration
     * @param check_collisions whether to check for collisions at each iteration
     *        @default true
     * @param return_approximate_solution whether to allow approximate solutions to the IK problem
     *        @default false
     * @param attempts number of times to try IK for each end-effector; the first time attempts to start from @p initial_guess, then uses random values
     *        @default 0, which means use default_ik_attempts_, internally defined or read from the parameter server
     * @param timeout timeout for each IK attempt
     *        @default 0.0, which means use default_ik_timeout_, internally defined or read from the parameter server
     * @param allowed_collisions user-specified allowed collisions (extra to the ones already present in the robot SRDF)
     *        @default empty
     * 
     * @return true on success
     */
    bool find_group_ik(std::string group_name, const std::vector< geometry_msgs::Pose >& ee_poses, std::vector< std::vector< double > >& solutions, const std::vector< double >& initial_guess = std::vector<double>(), bool check_collisions = true, bool return_approximate_solution = false, unsigned int attempts = 0, double timeout = 0.0, const std::map< std::string, std::string >& allowed_collisions = std::map< std::string, std::string >());
    
private:
    // ros variables
    ros::NodeHandle node;
    ros::ServiceClient ik_serviceClient_;
    ros::Subscriber scene_sub_;
  
    // MoveIt! variables
    planning_scene::PlanningScenePtr planning_scene_;
    robot_model::RobotModelPtr kinematic_model_;
    robot_state::RobotStatePtr kinematic_state_;
    collision_detection::CollisionRequest collision_request_;
    collision_detection::CollisionResult collision_result_;
    collision_detection::AllowedCollisionMatrix acm_;
    
    // utility variables
    bool is_initialized_ = false;
    std::map<std::string,std::string> group_map_;
    std::vector<std::string> chain_names_list_;
    std::vector<std::string> tree_names_list_;
    std::vector<std::string> group_names_;
    std::map<std::string,std::vector<std::string>> tree_composition_;
    std::mutex scene_mutex_;
    std::mutex map_mutex_;
    double default_ik_timeout_ = 0.005;
    unsigned int default_ik_attempts_ = 10;
    
    // managing external parameters
    XmlRpc::XmlRpcValue ik_control_params;
    
    /**
     * @brief utility function to initialize all class variables
     * 
     * @param kinematic_model
     *   a pointer to a robot model to be stored in the class and used to construct everything else
     * @return void
     */
    void initializeIKCheckCapability(const moveit::core::RobotModelPtr& kinematic_model);
    
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
     * @brief callback to update the planning scene using monitored_planning_scene
     * 
     * @return void
     */
    void scene_callback(const moveit_msgs::PlanningScene_< std::allocator< void > >::ConstPtr& plan_msg);
    
    /**
     * @brief function to find IK value for a given end-effector pose
     * 
     * @param ee_name name of the end-effector we want to find IK for
     * @param ee_pose desired pose for the end-effector
     * @param solution the solution found
     * @param initial_guess the starting point for the IK search
     *        @default empty vector, using the default robot configuration
     * @param check_collisions whether to check for collisions at each iteration
     *        @default true
     * @param return_approximate_solution whether to allow approximate solutions to the IK problem
     *        @default false
     * @param attempts number of times to try IK; the first time attempts to start from @p initial_guess, then uses random values
     *        @default 0, which means use default_ik_attempts_, internally defined or read from the parameter server
     * @param timeout timeout for each IK attempt
     *        @default 0.0, which means use default_ik_timeout_, internally defined or read from the parameter server
     * 
     * @return true on success
     */
    bool find_ik(std::string ee_name, const geometry_msgs::Pose& ee_pose, std::vector< double >& solution, const std::vector< double >& initial_guess = std::vector<double>(), bool check_collisions = true, bool return_approximate_solution = false, unsigned int attempts = 0, double timeout = 0.0);
    
    /**
     * @brief function to find IK value for a given set of end-effectors and respective poses
     * This function finds iterativelly IK solutions for each end-effector (if it's able to), using nested calls which try to solve for a chain, then use that solution to solve for another, and if it's not possible try again to solve for the previous - up to a maximum of attempts number of times (at most attempts^(chains.size()) IK calls can be performed in total).
     * This function does not allow for setting initial guesses, as the current robot state is used as a first attempt, and random values are used in the following ones.
     * 
     * @param chains vector of names of each end-effector
     * @param ee_poses desired poses for all the specified end-effectors
     * @param solutions the solutions found
     * @param ik_index the index of the first IK which needs to be solved by this instance of the function
     *        @default 0, i.e. start from the beginning
     * @param check_collisions whether to check for collisions at each iteration
     *        @default true
     * @param return_approximate_solution whether to allow approximate solutions to the IK problem
     *        @default false
     * @param attempts number of times to try IK for each end-effector; the first time attempts to start from @p initial_guess, then uses random values
     *        @default 0, which means use default_ik_attempts_, internally defined or read from the parameter server
     * @param timeout timeout for each IK attempt
     *        @default 0.0, which means use default_ik_timeout_, internally defined or read from the parameter server
     * 
     * @return true on success
     */
    bool find_ik(const std::vector<std::string>& chains, const std::vector< geometry_msgs::Pose >& ee_poses, std::vector< std::vector< double > >& solutions, unsigned int ik_index = 0, bool check_collisions = true, bool return_approximate_solution = false, unsigned int attempts = 0, double timeout = 0.0);
    
    /**
     * @brief function to test whether the current pose is collision free, considering both self-collisions and the current planning scene
     * 
     * @return true on success
     */
    bool is_collision_free(moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup* jmg, const double* q);
};

}
}

#endif // IKCHECKCAPABILITY_H
