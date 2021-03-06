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
    bool find_group_ik(std::string group_name, const geometry_msgs::Pose& ee_pose, std::vector< double >& solution, const std::vector< double >& initial_guess = std::vector<double>(), bool check_collisions = true, bool return_approximate_solution = false, unsigned int attempts = 0, double timeout = 0.0, const std::map< std::string, std::string >& allowed_collisions = std::map< std::string, std::string >());
    
    /**
     * @brief utility function to find the closest IK out of a number of trials; this calls find_group_ik that number of times (or until a distance threshold is respected) and stores the closest solution found
     * 
     * @param group_name name of the group we want to find IK for
     * @param ee_poses desired poses for all the end-effectors of the group
     * @param solutions the solutions found
     * @param it_info structure keeping information about all the iterations performed by this routine (returned empty if @p store_iterations is true)
     * @param store_iterations flag to say whether to store accurate information about each iteration (they will be stored in @p it_info
     *        @default false
     * @param allowed_distance threshold to consider a result valid (sum of absolute distances in joint space)
     *        @default 0.5
     * @param single_distances a value for the distance to be respected by single joints. this vector should be either empty or have a number of entries equal to the joints we are considering
     * @param trials_nr number of time (maximum) to evaluate the whole IK (max calls to find_group_ik)
     *        @default 5
     * @param initial_guess the starting point for the IK search (must be for the whole group)
     *        @default empty vector, using the default robot configuration
     * @param check_collisions whether to check for collisions at each iteration
     *        @default true
     * @param return_approximate_solution whether to allow approximate solutions to the IK problem
     *        @default false
     * @param attempts number of times to try IK for each end-effector in each trial; the first time attempts to start from @p initial_guess, then uses random values
     *        @default 0, which means use default_ik_attempts_, internally defined or read from the parameter server
     * @param timeout timeout for each IK attempt
     *        @default 0.0, which means use default_ik_timeout_, internally defined or read from the parameter server
     * @param use_clik whether to use CLIK
     * @param clik_percentage between 0 and 1, how much of the distance should be covered by CLIK in order to consider the result ok
     * @param allowed_collisions user-specified allowed collisions (extra to the ones already present in the robot SRDF)
     *        @default empty
     * 
     * @return true if a solution to the IK problem within the requested accuracy has been found, false otherwise; if any solution has been found, the best one is stored in solutions vector
     */
    bool find_closest_group_ik(std::string group_name, const geometry_msgs::Pose& ee_pose, std::vector< double >& solution, double allowed_distance = 0.5, std::vector<double> single_distances = std::vector<double>(), unsigned int trials_nr = 5, const std::vector< double >& initial_guess = std::vector<double>(), bool check_collisions = true, bool return_approximate_solution = false, unsigned int attempts = 0, double timeout = 0.0, const std::map< std::string, std::string >& allowed_collisions = std::map< std::string, std::string >());
    
    /**
     * @brief function to reset the internal robot state (or a group of its joints) to a given state
     * Reset the internal robot state (or a group of its joints) to a given state whose name is listed in the SRDF. If this function is called without arguments, the whole robot is reset to its default configuration
     * 
     * @param group name of the group
     * @param named_target name of the configuration (as given in the SRDF)
     * 
     * @return true on success
     */
    bool reset_robot_state(std::string group = std::string(), std::string named_target = std::string());
    
    /**
     * @brief function to reset the internal robot state to a given state
     * 
     * @param rs robot state to be used for the internal robot state
     * 
     * @return true on success
     */
    bool reset_robot_state(const moveit::core::RobotState& rs);
    
    /**
     * @brief function to set the internal robot state (or a group of its joints) to a given state
     * 
     * @param group name of the group
     * @param target values for all joints in the group
     * 
     * @return true on success
     */
    bool reset_robot_state(std::string group, std::vector<double> target);
    
    /**
     * @brief function to get a copy of the internal robot state
     * 
     * @return the internal robot state
     */
    moveit::core::RobotState get_robot_state();
    
    /**
     * @brief function to test whether a robot-state is collision free, considering both self-collisions and the current planning scene (unless @p self_collision_only is set to true)
     * 
     * @return true on success
     */
    bool is_state_collision_free(moveit::core::RobotState* robot_state, std::string group, bool self_collision_only);
    
    /**
     * @brief function to get a const pointer to the internal planning scene
     * 
     * @param updated flag saying whether to return the updated planning scene or the one which always stays empty
     * 
     * @return the internal robot state
     */
    planning_scene::PlanningSceneConstPtr get_planning_scene(bool updated);
    
private:
    // ros variables
    ros::NodeHandle node;
    ros::Subscriber scene_sub_;
    
    // MoveIt! variables
    planning_scene::PlanningScenePtr planning_scene_;
    planning_scene::PlanningScenePtr empty_planning_scene_;
    robot_model::RobotModelPtr kinematic_model_;
    robot_state::RobotStatePtr kinematic_state_;
    collision_detection::CollisionRequest collision_request_;
    collision_detection::CollisionResult collision_result_;
    collision_detection::AllowedCollisionMatrix acm_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    
    // utility variables
    bool is_initialized_ = false;
    bool kinematics_only_ = false;
    std::map<std::string,std::string> group_map_;
    std::vector<std::string> chain_names_list_;
    std::vector<std::string> tree_names_list_;
    std::vector<std::string> group_names_;
    std::map<std::string,std::vector<std::string>> tree_composition_;
    std::mutex scene_mutex_;
    std::mutex interface_mutex_;
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
     * @brief function to test whether the current pose is collision free, considering both self-collisions and the current planning scene
     * 
     * @return true on success
     */
    bool is_collision_free(moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup* jmg, const double* q);
    
    /**
     * @brief function to test whether the current pose is self-collision free
     * 
     * @return true on success
     */
    bool is_self_collision_free(moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup* jmg, const double* q);
    
    /**
     * @brief checks whether the group with name @p group_name can be managed by this class
     * 
     * @return true on success
     */
    bool can_be_managed(const std::string& group_name);
};

}
}

#endif // IKCHECKCAPABILITY_H
