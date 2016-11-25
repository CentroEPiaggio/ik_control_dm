#ifndef SLIDING_CAPABILITY_H_
#define SLIDING_CAPABILITY_H_

#include <dual_manipulation_ik_control_capabilities/generic_planning_capability.h>
#include <atomic>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <bezier_curve/bezier_curve.h>
#include <ik_check_capability/ik_check_capability.h>

namespace dual_manipulation
{
namespace ik_control
{

class SlidingCapability : public GenericPlanningCapability
{
public:
    SlidingCapability(shared_ik_memory& sikm_, const ros::NodeHandle& node_ = ros::NodeHandle());
    virtual ~SlidingCapability();
    virtual bool isComplete();
    virtual void performRequest(dual_manipulation_shared::ik_serviceRequest req);
    virtual bool getResults(dual_manipulation_shared::ik_response& res);
    virtual bool canRun();
    virtual bool canPerformCapability(const ik_control_capabilities& ik_capability) const;
    virtual void reset();
    void add_target(const dual_manipulation_shared::ik_service::Request& req);
    
private:
    shared_ik_memory& sikm;
    const ik_control_capability capabilities_;
    std::map<std::string, geometry_msgs::Pose > targets_;
    
    BezierCurve planner_bezier_curve;
    
    // MoveIt! variables
    moveit::core::RobotModelPtr robot_model_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    std::string robot_description;
    
    // ros variables
    ros::NodeHandle node;
    
    // interface and results variables
    std::atomic_bool busy;
    dual_manipulation_shared::ik_response response_;
    
    //parameters
    Eigen::Affine3d ee_contact;
    double fixed_translation_bezier; // this parameters is now set in parameter file but has to be studied
    
    KDL::Frame Object_PreSlide, Object_Slide;
    
private:
    /**
     * @brief utility function to parse parameters from the parameter server
     * 
     * @param params
     *   all useful params got from the parameter server
     * @return void
     */
    void parseParameters(XmlRpc::XmlRpcValue& params);
    
    /**
     * @brief utility function to set class variables which depend on parameters
     * 
     * @return void
     */
    void setParameterDependentVariables();
    void planSliding(const dual_manipulation_shared::ik_serviceRequest& req);
    void compute_orientation_from_vector(const Eigen::Vector3d& x_new, Eigen::Quaterniond& res);
};

}
}

#endif // SLIDING_CAPABILITY_H_
