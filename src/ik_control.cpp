#include "ik_control.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <thread>

using namespace dual_manipulation::ik_control;

ikControl::ikControl()
{
    busy["left_hand"]=false;
    busy["right_hand"]=false;

    trj_gen["left_hand"] = new dual_manipulation::ik_control::trajectory_generator();
    trj_gen["right_hand"] = new dual_manipulation::ik_control::trajectory_generator();
    
    hand_pub["left_hand"] = node.advertise<std_msgs::String>("/ik_control/left_hand/action_done",0,this);
    hand_pub["right_hand"] = node.advertise<std_msgs::String>("/ik_control/right_hand/action_done",0,this);
    
    isInitialized_ = false;
}
    
void ikControl::initializeRobotModel(const urdf::Model *const robot_model)
{
    if (isInitialized_)
    {
      ROS_WARN("IKControl: robot model already initialized - NOT initializing it again...");
      return;
    }
    
    // initialize robot kinematics
    ROS_INFO("IKControl: Init kinematic tree, chains, solvers and joint arrays.");
    if (!kdl_parser::treeFromUrdfModel(*robot_model, robot_tree_))
    {
      ROS_ERROR("Failed to construct kdl tree");
      return;
    }

    ROS_INFO("Robot kinematics successfully parsed with %d joints, and %d segments.",robot_tree_.getNrOfJoints(),robot_tree_.getNrOfSegments() );

    std::string root_name = robot_tree_.getRootSegment()->first;

    // KINEMATICS
    // NOTE:
    // KDL Calculates the jacobian expressed in the base frame of the chain, with reference point at the end effector of the *chain
    // The total wrench at links are expressed in the link frame, using the origin as the reference point
    // The resultant joint effort due to contact is reference frame independent, however, the MultiplyJacobian must be consistent in reference frames.

    // Get the chains for right and left hand
    robot_tree_.getChain(root_name, "left_hand_palm_link",  base_to_left_hand_chain_);
    robot_tree_.getChain(root_name, "right_hand_palm_link", base_to_right_hand_chain_);
    
    // NOTE:
    // joints to control the synergy are named {hand_name}_synergy_joint, but cannot be used here: they need separate kinematic chains as the {hand_name}_palm_joint has the same parent

    // init the jacobian solvers
    base_to_left_hand_jac_solver_  = new KDL::ChainJntToJacSolver(base_to_left_hand_chain_);
    base_to_right_hand_jac_solver_ = new KDL::ChainJntToJacSolver(base_to_right_hand_chain_);

    // init the forward kinematic solvers
    base_to_left_hand_fk_solver_  = new KDL::ChainFkSolverPos_recursive(base_to_left_hand_chain_);
    base_to_right_hand_fk_solver_ = new KDL::ChainFkSolverPos_recursive(base_to_right_hand_chain_);
    
    left_hand_arm_dof_nr_  = base_to_left_hand_chain_.getNrOfJoints();
    right_hand_arm_dof_nr_ = base_to_right_hand_chain_.getNrOfJoints();

    // resize the complete jntarrays
    left_hand_arm_joints_  = KDL::JntArray( left_hand_arm_dof_nr_ );
    right_hand_arm_joints_ = KDL::JntArray( right_hand_arm_dof_nr_ );

    // init the jacobians
    base_to_left_hand_jac_ =  KDL::Jacobian( left_hand_arm_dof_nr_ );
    base_to_right_hand_jac_ = KDL::Jacobian( right_hand_arm_dof_nr_ );
    
    // set to zero initial positions and velocities
    left_hand_arm_joint_names_.resize( left_hand_arm_dof_nr_ );
    left_hand_arm_link_names_.resize( left_hand_arm_dof_nr_ );
    left_hand_arm_joint_position_.resize( left_hand_arm_dof_nr_ );
    left_hand_arm_joint_velocity_.resize( left_hand_arm_dof_nr_ );
    for(unsigned int j=0; j < left_hand_arm_dof_nr_; ++j)
    {
      left_hand_arm_joint_names_[j] = base_to_left_hand_chain_.getSegment(j).getJoint().getName();
      left_hand_arm_link_names_[j] = base_to_left_hand_chain_.getSegment(j).getName();
      left_hand_arm_joint_position_[j] = 0.0;
      left_hand_arm_joint_velocity_[j] = 0.0;
    }
    right_hand_arm_joint_names_.resize( right_hand_arm_dof_nr_ );
    right_hand_arm_link_names_.resize( right_hand_arm_dof_nr_ );
    right_hand_arm_joint_position_.resize( right_hand_arm_dof_nr_ );
    right_hand_arm_joint_velocity_.resize( right_hand_arm_dof_nr_ );
    for(unsigned int j=0; j < right_hand_arm_dof_nr_; ++j)
    {
      right_hand_arm_joint_names_[j] = base_to_right_hand_chain_.getSegment(j).getJoint().getName();
      right_hand_arm_link_names_[j] = base_to_right_hand_chain_.getSegment(j).getName();
      right_hand_arm_joint_position_[j] = 0.0;
      right_hand_arm_joint_velocity_[j] = 0.0;
    }
    
    std::cout << "IKControl: Robot model initizalization done!" << std::endl;
    isInitialized_ = true;

    // just to test that everything goes well at loading time
    updateKinematics();

}

void ikControl::updateKinematics()
{
    if (!isInitialized_)
    {
      ROS_WARN("IKControl::updateKinematics: robot model is not initialized - initialize it first!");
      return;
    }
    
    // update joint arrays
    for(int i=0; i<left_hand_arm_joint_position_.size() ; ++i)
    {
      left_hand_arm_joints_(i) = left_hand_arm_joint_position_[i];
    }
    for(int i=0; i<right_hand_arm_joint_position_.size() ; ++i)
    {
      right_hand_arm_joints_(i) = right_hand_arm_joint_position_[i];
    }

    // update jacobians
    base_to_left_hand_jac_solver_->JntToJac(left_hand_arm_joints_, base_to_left_hand_jac_);
    base_to_right_hand_jac_solver_->JntToJac(right_hand_arm_joints_, base_to_right_hand_jac_);

    // update frames
    base_to_left_hand_fk_solver_->JntToCart(left_hand_arm_joints_, left_hand_frame_);
    base_to_right_hand_fk_solver_->JntToCart(right_hand_arm_joints_, right_hand_frame_);

//     // update jacobian with chage of frame
//     base_to_left_hand_jac_.changeRefFrame(left_hand_frame_);
//     base_to_right_hand_jac_.changeRefFrame(right_hand_frame_);

    // print to test
    std::cout << "left_hand_frame_ : " << std::endl;
    std::cout << left_hand_frame_.M.data[0] << " " << left_hand_frame_.M.data[1] << " " << left_hand_frame_.M.data[2] << " " << left_hand_frame_.p.data[0] << std::endl;
    std::cout << left_hand_frame_.M.data[3] << " " << left_hand_frame_.M.data[4] << " " << left_hand_frame_.M.data[5] << " " << left_hand_frame_.p.data[1] << std::endl;
    std::cout << left_hand_frame_.M.data[6] << " " << left_hand_frame_.M.data[7] << " " << left_hand_frame_.M.data[8] << " " << left_hand_frame_.p.data[2] << std::endl;
    std::cout << "right_hand_frame_: " << std::endl;
    std::cout << right_hand_frame_.M.data[0] << " " << right_hand_frame_.M.data[1] << " " << right_hand_frame_.M.data[2] << " " << right_hand_frame_.p.data[0] << std::endl;
    std::cout << right_hand_frame_.M.data[3] << " " << right_hand_frame_.M.data[4] << " " << right_hand_frame_.M.data[5] << " " << right_hand_frame_.p.data[1] << std::endl;
    std::cout << right_hand_frame_.M.data[6] << " " << right_hand_frame_.M.data[7] << " " << right_hand_frame_.M.data[8] << " " << right_hand_frame_.p.data[2] << std::endl;
    
}

void ikControl::ik_thread(dual_manipulation_shared::ik_service::Request req)
{
    if (!isInitialized_)
    {
      ROS_WARN("IKControl::ik_thread: robot model is not initialized - initialize it first!");
      return;
    }
    
    KDL::Frame start_pose,final_pose;
    KDL::Frame next_pose;
    KDL::Twist next_vel;
    
    ROS_INFO("Thread spawned! Computing ik for %s",req.ee_name.c_str());
    
    //TODO: SENSE ee cartesian position to set start, using TF
    
    start_pose.p = KDL::Vector::Zero();
    start_pose.M = KDL::Rotation::Identity();
    
    tf::poseMsgToKDL(req.ee_pose,final_pose);
    
    trj_gen[req.ee_name]->initialize_line_trajectory(req.time, start_pose, final_pose); //initializing trajectory
    
    
    
    ros::Time start_t, now;
    ros::Duration final_t;
    
    final_t.fromNSec(req.time*1000000000.0);
    
    start_t = ros::Time::now();
    while(now-start_t<final_t)
    {
        now = ros::Time::now();
      
	trj_gen[req.ee_name]->line_trajectory(((now-start_t).toNSec()/1000000000.0), next_pose, next_vel); //computing next pose
	
	//TODO: PERFORM trajectory
	
	static tf::TransformBroadcaster br;
	tf::Transform current_robot_transform;
	tf::transformKDLToTF(next_pose,current_robot_transform);
	br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "base_link", req.ee_name.c_str()));
    }
  
    msg.data = "done";
    hand_pub[req.ee_name].publish(msg); //publish on a topic when the trajectory is done
  
    busy[req.ee_name]=false;
    
    return;
}

bool ikControl::perform_ik(dual_manipulation_shared::ik_service::Request& req)
{
    if (!isInitialized_)
    {
      ROS_WARN("IKControl::perform_ik: robot model is not initialized - initialize it first!");
      return false;
    }

    if(!busy.count(req.ee_name))
    {
	ROS_ERROR("Unknown end effector %s, returning",req.ee_name.c_str());
	return false;
    }
    if(!busy[req.ee_name])
    {
	busy[req.ee_name]=true;
	std::thread* th = new std::thread(&ikControl::ik_thread,this, req);
    }
    else
    {
	ROS_WARN("Already performing a %s IK",req.ee_name.c_str());
	return false;
    }
    
    

    return true;
}

ikControl::~ikControl()
{
    delete trj_gen["left_hand"];
    delete trj_gen["right_hand"];
}
