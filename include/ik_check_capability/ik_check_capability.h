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

typedef std::pair<double,std::vector<std::vector<double>>> ik_iteration_info;

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
    
    /**
     * @brief function to perform Closed-Loop IK
     * This functions try to get as close as possible to the required pose, interpolating from current to desired. The step to go towards the desired pose, initially big, is gradually decreased to get a finer solution. This function stops when the step is less than 0.5% of the initial gap, or when the pose has been reached.
     * 
     * @param group_name name of the group we want to find IK for
     * @param ee_poses desired poses for all the end-effectors of the group
     * @param solutions the solutions found
     * @param initial_guess the starting point for the IK search (must be for the whole group)
     *        @default empty vector, using the default robot configuration
     * @param check_collisions whether to check for collisions at each iteration
     *        @default true
     * @param attempts number of times to try IK; the first time attempts to start from @p initial_guess, then uses random values
     *        @default 0, which means use default_ik_attempts_, internally defined or read from the parameter server
     * @param timeout timeout for each IK attempt
     *        @default 0.0, which means use default_ik_timeout_, internally defined or read from the parameter server
     * @param allowed_collisions user-specified allowed collisions (extra to the ones already present in the robot SRDF)
     *        @default empty
     * 
     * @return percentage of the cartesia deviation between current and desired configurations which has been found feasible (always between 0 and 1)
     */
    double clik(std::string group_name, const std::vector< geometry_msgs::Pose >& ee_poses, std::vector< std::vector< double > >& solutions, const std::vector< double >& initial_guess = std::vector<double>(), bool check_collisions = true, unsigned int attempts = 0, double timeout = 0.0, const std::map< std::string, std::string >& allowed_collisions = std::map< std::string, std::string >());
  
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
     * @param allowed_collisions user-specified allowed collisions (extra to the ones already present in the robot SRDF)
     *        @default empty
     * 
     * @return true if a solution to the IK problem within the requested accuracy has been found, false otherwise; if any solution has been found, the best one is stored in solutions vector
     */
    bool find_closest_group_ik(std::string group_name, const std::vector< geometry_msgs::Pose >& ee_poses, std::vector< std::vector< double > >& solutions, std::vector<ik_iteration_info>& it_info, bool store_iterations = false, double allowed_distance = 0.5, unsigned int trials_nr = 5, const std::vector< double >& initial_guess = std::vector<double>(), bool check_collisions = true, bool return_approximate_solution = false, unsigned int attempts = 0, double timeout = 0.0, const std::map< std::string, std::string >& allowed_collisions = std::map< std::string, std::string >());
    
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
     * @brief implementation of find_group_ik interface function, used also from find_closest_group_ik call
     * This function implements the interface of find_group_ik and find_closest_group_ik functions; all parameters are the same as described in those functions, except for @p jmg and @p chains
     * 
     * @param jmg joint model of the whole group, needed in order to correctly set the start state for the IK procedure
     * @param chains vector of names of each end-effector
     * 
     * @return true on success
     */
    bool find_group_ik_impl(const moveit::core::JointModelGroup* jmg, const std::vector< std::string >& chains, const std::vector< geometry_msgs::Pose >& ee_poses, std::vector< std::vector< double > >& solutions, const std::vector< double >& initial_guess, bool check_collisions, bool return_approximate_solution, unsigned int attempts, double timeout);
    
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
     * @brief function to get the interpolation (K in [0,1]) between certain link positions of a given robot configuration and an equal number of frames
     * 
     * @param rs robot state from which getting the link positions
     * @param links vector of links of the robot to get the interpolation of
     * @param des_poses vector of desired poses of such links
     * @param K interpolation parameter between 0 (meaning current configuration) and 1 (meaning desired pose)
     * @param interp_poses vector of interpolated poses
     */
    void computeCartesianErrors(const moveit::core::RobotStatePtr& rs, const std::vector< const moveit::core::LinkModel* >& links, const std::vector< geometry_msgs::Pose >& des_poses, double K, std::vector< geometry_msgs::Pose >& interp_poses);
};

}
}

#endif // IKCHECKCAPABILITY_H
