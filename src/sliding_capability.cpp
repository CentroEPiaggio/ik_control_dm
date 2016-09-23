#include <dual_manipulation_ik_control_capabilities/sliding_capability.h>

#include <eigen3/Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#define CLASS_NAMESPACE "ikControl::slidingCapability::"
#define CLASS_LOGNAME "ikControl::slidingCapability"
#define FIXED_TRANSLATION_BEZIER 0.1
#include <trajectory_utils.h>
#include <dual_manipulation_shared/parsing_utils.h>

using namespace dual_manipulation::ik_control;

SlidingCapability::SlidingCapability(shared_ik_memory& sikm_, const ros::NodeHandle& node_) : sikm(sikm_), node(node_)
{
    reset();
}

SlidingCapability::~SlidingCapability()
{

}

bool SlidingCapability::canPerformCapability(const ik_control_capabilities& ik_capability) const
{
    //TODO
    return true;
}


void SlidingCapability::reset()
{
    abstractCapability::reset();
    {
        std::unique_lock<std::mutex> ul(sikm.m);
        parseParameters(*sikm.ik_control_params);
    }
    robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader(robot_description));
    robot_model_ = robot_model_loader_->getModel();
    ik_check_.reset(new ikCheckCapability(robot_model_));
    busy.store(false);
}


void SlidingCapability::parseParameters(XmlRpc::XmlRpcValue& params)
{
    parseSingleParameter(params,robot_description,"robot_description");
}

void SlidingCapability::setParameterDependentVariables()
{

}

bool SlidingCapability::canRun()
{
    return true;
}

bool SlidingCapability::isComplete()
{
    return true;
}

bool SlidingCapability::getResults(dual_manipulation_shared::ik_response& res)
{
    res = response_;
    return response_.data=="done";
}

void SlidingCapability::performRequest(dual_manipulation_shared::ik_serviceRequest req)
{
    bool I_am_busy = busy.exchange(true);
    if(I_am_busy)
        return;
    
    planSliding(req);

}

void SlidingCapability::planSliding(const dual_manipulation_shared::ik_serviceRequest& req)
{
    ros::Time t;
    while(!sikm.getNextTrajectoyEndTime(t))
    {
        usleep(5000);
    }
    
    response_.seq = req.seq;
    response_.group_name = req.ee_name;
    
    geometry_msgs::Pose goal_pose = req.ee_pose.at(0);
    
    std::string group_name;
    if (!sikm.groupManager->getGroupInSRDF(req.ee_name, group_name))
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : no group named " << req.ee_name);
        response_.data = "error";
        return;
    }
    
    moveit::core::RobotState rs = sikm.getPlanningRobotState();    
    std::string ee_link_name = robot_model_->getJointModelGroup(group_name)->getEndEffectorParentGroup().second;
    
    // initial end-effector pose expressed in world frame
    Eigen::Affine3d world_ee = rs.getFrameTransform(ee_link_name);
    Eigen::Affine3d ee_contact;
    ee_contact = Eigen::Translation3d(0.0,0.0,0.20); //TODO manage this prior to inverse kinematics
    Eigen::Affine3d init_contact_pose = world_ee * ee_contact;
    
    Eigen::Vector3d x_i = world_ee.rotation().leftCols(1);
    Eigen::Affine3d goal_pose_eigen;
    tf::poseMsgToEigen(goal_pose, goal_pose_eigen);
    // the plane to use for sliding
    Eigen::Matrix<double,3,2> projection_plane;
    projection_plane = goal_pose_eigen.rotation().leftCols(2);
    Eigen::Vector3d x_projected = projection_plane*projection_plane.transpose() * x_i;
    //norm of x_projected
    double n_x_projected = x_projected.norm();
    if (n_x_projected <= 1e-3)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : sliding axis orthogonal to sliding plane");
        response_.data = "error";
        return;
    }
    x_projected /= n_x_projected;
    
    BezierCurve::PointVector point_for_planning;
    point_for_planning.resize(point_for_planning.RowsAtCompileTime, 4);
       
    // a point close to the init point
    BezierCurve::PointVector aux_point_1 = init_contact_pose.translation() + x_projected*FIXED_TRANSLATION_BEZIER;
    // a point close to the goal point
    BezierCurve::PointVector aux_point_2 = goal_pose_eigen.translation() - projection_plane.leftCols(1)*FIXED_TRANSLATION_BEZIER;
    
    point_for_planning << init_contact_pose.translation(), aux_point_1 , aux_point_2, goal_pose_eigen.translation();
    
    planner_bezier_curve.init_curve(point_for_planning);
    
    int num_samples = 20;
    
    std::vector <geometry_msgs::Pose > waypoints;
    
    Eigen::Affine3d contact_ee_init;
    for(double s_bezier = 0.0; s_bezier<=1.0; s_bezier+=1.0/num_samples)
    {
        BezierCurve::Point res = planner_bezier_curve.compute_point(s_bezier);
        Eigen::Quaterniond rot;
        BezierCurve::Point dres = planner_bezier_curve.compute_derivative(s_bezier);  
        compute_orientation_from_vector(dres, rot);
        Eigen::Affine3d eigen_waypoint = Eigen::Translation3d(res)*rot;
        if(s_bezier == 0.0)
            contact_ee_init = eigen_waypoint.inverse()*world_ee;
        geometry_msgs::Pose newWaypoint;
        tf::poseEigenToMsg(eigen_waypoint*contact_ee_init, newWaypoint);
        waypoints.push_back(newWaypoint);
    }
    moveit_msgs::RobotTrajectory planned_joint_trajectory;
    std::vector<double> single_distances({0.5,0.5,0.5,1.0,2.0,2.0,2.0});
    double completed = computeTrajectoryFromWPs(planned_joint_trajectory, waypoints, *ik_check_, group_name, req.ee_name, false, 2.5,single_distances);
    
    if(completed != 1.0)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to get trajectory from waypoints, returning");
        response_.data = "error";
        return;
    }
    
    sikm.swapTrajectory(req.ee_name, planned_joint_trajectory);
    response_.data = "done";
}

void SlidingCapability::compute_orientation_from_vector(const Eigen::Vector3d& x_new, Eigen::Quaterniond& res)
{
    // double x_norm = x_new.Norm();
    double x_norm = x_new.dot(x_new);
    if (x_norm < 1e-16)
    {
        ROS_WARN_STREAM("The norm of the required ax is close to zero: not performing the rotation");
        res.setIdentity();
        return;
    }
    
    Eigen::Vector3d x_ax(1,0,0);
    res.setFromTwoVectors(x_ax,x_new);
}

