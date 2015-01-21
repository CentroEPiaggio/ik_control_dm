#ifndef IK_CONTROL_H
#define IK_CONTROL_H

#include "trajectory_generator.h"
#include "dual_manipulation_shared/ik_service.h"
#include <std_msgs/String.h>

// URDF
#include <urdf/model.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace dual_manipulation
{
namespace ik_control
{
  
/**
  * @brief This is a class that is used from the ros_server to perform a desired ik using a dedicated thread (one for each end effector).
  * 
  */
class ikControl
{
public:
    ikControl();
    ~ikControl();
    
    /**
     * @brief interface function to perform the @e ik_service
     * 
     * @param req
     *   the req from the @e ik_service
     * @return bool
     */
    bool perform_ik(dual_manipulation_shared::ik_service::Request &req);

    /**
     * @brief function to initialize the robot model, necessary to perform IK and trajectory generation
     * 
     * @param robot_model
     *   urdf::Model containing the robot model
     * @return void
     */
    void initializeRobotModel(const urdf::Model *const );
    
private:
    // initialization variable
    bool isInitialized_;
    // robot tree
    KDL::Tree robot_tree_;
    // link and joint names and values
    unsigned int left_hand_arm_dof_nr_;
    std::vector<std::string> left_hand_arm_link_names_;
    std::vector<std::string> left_hand_arm_joint_names_;
    std::vector<double> left_hand_arm_joint_position_;
    std::vector<double> left_hand_arm_joint_velocity_;
    unsigned int right_hand_arm_dof_nr_;
    std::vector<std::string> right_hand_arm_link_names_;
    std::vector<std::string> right_hand_arm_joint_names_;
    std::vector<double> right_hand_arm_joint_position_;
    std::vector<double> right_hand_arm_joint_velocity_;
    // KDL chains and solvers
    KDL::JntArray left_hand_arm_joints_; // only the abduction, no mimic
    KDL::JntArray right_hand_arm_joints_; // abduction + inner + mimic
    KDL::Chain base_to_left_hand_chain_;
    KDL::Chain base_to_right_hand_chain_;
    KDL::ChainJntToJacSolver* base_to_left_hand_jac_solver_;
    KDL::ChainJntToJacSolver* base_to_right_hand_jac_solver_;
    KDL::ChainFkSolverPos_recursive* base_to_left_hand_fk_solver_;
    KDL::ChainFkSolverPos_recursive* base_to_right_hand_fk_solver_;
    KDL::Jacobian base_to_left_hand_jac_;
    KDL::Jacobian base_to_right_hand_jac_;
    KDL::Frame left_hand_frame_;
    KDL::Frame right_hand_frame_;
  
    std::map<std::string,dual_manipulation::ik_control::trajectory_generator*> trj_gen;
    std::map<std::string,bool> busy;
    ros::NodeHandle node;
    std::map<std::string,ros::Publisher> hand_pub;
    std_msgs::String msg;

    /**
     * @brief this is the thread body, trajectory generation and ik control are performed in it
     * 
     * @param req
     *   the same req from the @e ik_service
     * @return void
     */
    void ik_thread(dual_manipulation_shared::ik_service::Request req);
    
    /**
     * @brief update the robot kinematics
     * 
     * @return void
     */
    void updateKinematics();
    
};

}
}

#endif // IK_CONTROL_H
