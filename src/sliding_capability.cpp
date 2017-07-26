#include <dual_manipulation_ik_control_capabilities/sliding_capability.h>

#include <eigen3/Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>
#define CLASS_NAMESPACE "ikControl::slidingCapability::"
#define CLASS_LOGNAME "ikControl::slidingCapability"
#include <dual_manipulation_shared/parsing_utils.h>
#define DEBUG_STRING {std::cout << CLASS_NAMESPACE << __func__ << "@" << __LINE__ << std::endl;}
#define DEBUG_VISUAL 0
#define DEBUG 0

#define PI 3.14159
#define SQRT2 1.41421

//TODO to delete
// for debugging purposes only
#if DEBUG_VISUAL
#include <rviz_visual_tools/rviz_visual_tools.h>
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    void publushStuff(const std::vector<geometry_msgs::Pose>& pose_msgs)
    {    
        if(!visual_tools_)
        {
            visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("vito_anchor","/rviz_visual_markers"));
        }
        
        for(auto& pose:pose_msgs)
        {
            visual_tools_->publishAxis(pose);
            ros::Duration(0.1).sleep();
        }
        

    }
#endif


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
    if (ik_control_capabilities::PLAN_SLIDE == ik_capability)
        return true;
    return false;
}


void SlidingCapability::reset()
{
    abstractCapability::reset();
    parseParameters(sikm.getIkControlParams());
    
    robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader(robot_description));
    robot_model_ = robot_model_loader_->getModel();
    busy.store(false);
    targets_.clear();
}

bool SlidingCapability::set_hand_pose_sliding(geometry_msgs::Pose source_position, geometry_msgs::Pose target_position, int current_source_grasp)
{
    ros::NodeHandle node;
    std::vector<double> pOP(6,0), pOS(6,0); //Parameters for Object_Preslide and Object_Slide
    bool use_slide =  true;

    // Here we check if the yaw angles of the two objects differ more than 45° -> if so, send warning
    KDL::Frame source_frame;
    tf::poseMsgToKDL(source_position, source_frame);
    KDL::Frame target_frame;
    tf::poseMsgToKDL(target_position, target_frame);
    double roll_s, pitch_s, yaw_s;
    double roll_t, pitch_t, yaw_t;
    source_frame.M.GetRPY(roll_s, pitch_s, yaw_s);
    target_frame.M.GetRPY(roll_t, pitch_t, yaw_t);

#if DEBUG
    if(std::abs(yaw_t - yaw_s)>= PI/4){
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " Source and Target differ in yaw more than 45°, sliding might not function correctly because of current IK limitations.");
    }
#endif

    // extracting x_s, y_s, -x_s, -y_s vectors of source object frame and the diff_v vector (difference between two object positions)
    KDL::Vector x_s = source_frame.M.UnitX();
    KDL::Vector _x_s = x_s;
    _x_s.ReverseSign();
    KDL::Vector y_s = source_frame.M.UnitY();
    KDL::Vector _y_s = y_s;
    _y_s.ReverseSign();
    KDL::Vector diff_v = target_frame.p - source_frame.p;

    // Defining the 4 dot products between diff_v and x_s, y_s, -x_s, -y_s vectors
    double xdot = dot(diff_v, x_s);
    double _xdot = dot(diff_v, _x_s);
    double ydot = dot(diff_v, y_s);
    double _ydot = dot(diff_v, _y_s);

    // choose the most negative dot product and set the hand pose but first check the correct side (sCbt or sCtt)
    // case 1: current source grasp is sCbt
    if(current_source_grasp == sCbt || current_source_grasp == sCbe)
    {
    if(xdot < _xdot && xdot < ydot && xdot < _ydot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_L", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_L", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else if(_xdot < xdot && _xdot < ydot && _xdot < _ydot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_R", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_R", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else if(ydot < xdot && ydot < _xdot && ydot < _ydot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_D", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_D", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else if(_ydot < xdot && _ydot < ydot && _ydot < _xdot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_U", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_U", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else{
        std::cout << CLASS_NAMESPACE << __func__ << " These source and target poses are quite strange for sliding. There might be some problems." << std::endl;
        return false;
    }
    }
    // case 2: current source grasp is sCtt
    else if(current_source_grasp == sCtt || current_source_grasp == sCte)
    {
    if(xdot < _xdot && xdot < ydot && xdot < _ydot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_L_2", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_L_2", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else if(_xdot < xdot && _xdot < ydot && _xdot < _ydot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_R_2", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_R_2", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else if(ydot < xdot && ydot < _xdot && ydot < _ydot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_D_2", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_D_2", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else if(_ydot < xdot && _ydot < ydot && _ydot < _xdot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_U_2", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_U_2", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else{
        std::cout << CLASS_NAMESPACE << __func__ << " These source and target poses are quite strange for sliding. There might be some problems." << std::endl;
        return false;
    }
    }
}

bool SlidingCapability::set_hand_pose_tilting(geometry_msgs::Pose source_position, geometry_msgs::Pose target_position, int current_source_grasp)
{
    ros::NodeHandle node;
    std::vector<double> pOP(6,0), pOS(6,0); //Parameters for Object_Pretilt and Object_Tilt
    bool use_tilt =  true;

    // Here we check if the yaw angles of the two objects differ more than 45° -> if so, send warning
    KDL::Frame source_frame;
    tf::poseMsgToKDL(source_position, source_frame);
    KDL::Frame target_frame;
    tf::poseMsgToKDL(target_position, target_frame);
    double roll_s, pitch_s, yaw_s;
    double roll_t, pitch_t, yaw_t;
    source_frame.M.GetRPY(roll_s, pitch_s, yaw_s);
    target_frame.M.GetRPY(roll_t, pitch_t, yaw_t);

    // extracting x_s, y_s, -x_s, -y_s vectors of source object frame and the diff_v vector (difference between two object positions)
    KDL::Vector x_s = source_frame.M.UnitX();
    KDL::Vector _x_s = x_s;
    _x_s.ReverseSign();
    KDL::Vector y_s = source_frame.M.UnitY();
    KDL::Vector _y_s = y_s;
    _y_s.ReverseSign();
    KDL::Vector diff_v = target_frame.p - source_frame.p;

    // Defining the 4 dot products between diff_v and x_s, y_s, -x_s, -y_s vectors
    double xdot = dot(diff_v, x_s);
    double _xdot = dot(diff_v, _x_s);
    double ydot = dot(diff_v, y_s);
    double _ydot = dot(diff_v, _y_s);

    // choose the most negative dot product and set the hand pose but first check the correct side (sCbt or sCtt)
    // case 1: current source grasp is sCbt
    if(current_source_grasp == sCbt || current_source_grasp == sCbe)
    {
    if(xdot < _xdot && xdot < ydot && xdot < _ydot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_L", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_L", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 1: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else if(_xdot < xdot && _xdot < ydot && _xdot < _ydot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_R", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_R", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 2: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else if(ydot < xdot && ydot < _xdot && ydot < _ydot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_D", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_D", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 3: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else if(_ydot < xdot && _ydot < ydot && _ydot < _xdot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_U", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_U", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 4: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else{
        std::cout << CLASS_NAMESPACE << __func__ << " These source and target poses are quite strange for tilting. There might be some problems." << std::endl;
        return false;
    }
    }
    // case 2: current source grasp is sCtt
    else if(current_source_grasp == sCtt || current_source_grasp == sCte)
    {
    if(xdot < _xdot && xdot < ydot && xdot < _ydot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_L_2", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_L_2", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 5: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else if(_xdot < xdot && _xdot < ydot && _xdot < _ydot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_R_2", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_R_2", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 6: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else if(ydot < xdot && ydot < _xdot && ydot < _ydot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_D_2", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_D_2", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 7: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else if(_ydot < xdot && _ydot < ydot && _ydot < _xdot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_U_2", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_U_2", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 8: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else{
        std::cout << CLASS_NAMESPACE << __func__ << " These source and target poses are quite strange for tilting. There might be some problems." << std::endl;
        return false;
    }
    }
}


void SlidingCapability::parseParameters(XmlRpc::XmlRpcValue& params)
{
    parseSingleParameter(params,robot_description,"robot_description");
    std::vector<double> peec(6,0);
    if(!parseSingleParameter(params["slide"],peec, "ee_contact",6))
    {
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " Parameter ee_contact not set. We use Identity()");
    }
    
    KDL::Frame ee_contact_kdl = KDL::Frame(KDL::Rotation::EulerZYX(peec.at(3),peec.at(4),peec.at(5)), KDL::Vector(peec.at(0),peec.at(1),peec.at(2)));
    
    tf::transformKDLToEigen(ee_contact_kdl, ee_contact);
    
    fixed_translation_bezier = .1;
    parseSingleParameter(params["slide"], fixed_translation_bezier, "fixed_translation_bezier");
    
    std::vector<double> pOP(6,0), pOS(6,0); //Parameters for Object_Preslide and Object_Slide
    bool use_slide =  true;
    use_slide &= parseSingleParameter(params["slide"], pOP, "Object_PreSlide_L", 6);
    use_slide &= parseSingleParameter(params["slide"], pOS, "Object_Slide_L", 6);
        
    if( use_slide )
    {
        Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
        Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
    }
    else    
        std::cout << CLASS_NAMESPACE << __func__ << " Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
    
}

void SlidingCapability::setParameterDependentVariables()
{

}

bool SlidingCapability::canRun()
{
    return !busy.load();
}

bool SlidingCapability::isComplete()
{
    return !busy.load();
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
    busy.store(false);

}

void SlidingCapability::planSliding(const dual_manipulation_shared::ik_serviceRequest& req)
{
    #if DEBUG>1
    std::cout<<"Starting ee_pose is: "<<std::endl<<req.obj_poses[0].position<<std::endl;
    std::cout<<"Arriving ee_pose is: "<<std::endl<<req.obj_poses[1].position<<std::endl;
    #endif

    // Setting the correct hand pose for sliding
    bool hand_positioning = set_hand_pose_sliding(req.obj_poses[0], req.obj_poses[1], req.current_source_grasp_id);
    if (!hand_positioning){
        std::cout << CLASS_NAMESPACE << __func__ << " Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
    }

    // Setting the correct hand pose for tilting
    bool tilt_positioning = set_hand_pose_tilting(req.obj_poses[0], req.obj_poses[1], req.current_source_grasp_id);
    if (!tilt_positioning){
        std::cout << CLASS_NAMESPACE << __func__ << " Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
    }

    ros::Time t;
    while(!sikm.getNextTrajectoyEndTime(t))
    {
        usleep(5000);
    }
    
    if(!targets_.count(req.ee_name))
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : No target for slide on " << req.ee_name);
        response_.data = "error";
        return;
    }
    
    response_.seq = req.seq;
    response_.group_name = req.ee_name;
    
    geometry_msgs::Pose goal_pose = targets_.at(req.ee_name);
    targets_.erase(req.ee_name);
    
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
    Eigen::Affine3d init_contact_pose = world_ee * ee_contact;
    
    Eigen::Vector3d x_i = world_ee.rotation().leftCols(1);
    Eigen::Affine3d goal_pose_eigen;
    tf::poseMsgToEigen(goal_pose, goal_pose_eigen);
    
    goal_pose_eigen = goal_pose_eigen*ee_contact;
    // the plane to use for sliding
    Eigen::Matrix<double,3,2> projection_plane;

    // FOR TILTING
    // Source and Target poses later used for correct computation of bezier arc control points
    KDL::Frame source_frame_tmp;
    tf::poseMsgToKDL(req.obj_poses[0], source_frame_tmp);
    KDL::Frame target_frame_tmp;
    tf::poseMsgToKDL(req.obj_poses[1], target_frame_tmp);
    //-----------
    
    Eigen::Vector3d x_pp = goal_pose_eigen.translation() - init_contact_pose.translation();
    double n_x_pp = x_pp.norm();
    if (n_x_pp < 1e-6)
    {
        x_pp << 1., 0., 0.;
    }
    else
    {
        x_pp /= n_x_pp;
    }
    projection_plane.block<3,1>(0,0) = x_pp;

    // If transition is of type TILT then vector for vertical plane is computed else usual vector sliding plane
    Eigen::Vector3d y_t;
    
    if(req.current_transition_tilting){
        y_t = goal_pose_eigen.rotation().block<3,1>(0,2);
    }
    else{
        y_t = goal_pose_eigen.rotation().block<3,1>(0,1);
    }
    //--------------

    Eigen::Vector3d y_t_2 = y_t;
    y_t = y_t_2 - x_pp*(y_t_2.dot(x_pp));
    
    if (y_t.norm() > 1e-6)
    {
        projection_plane.block<3,1>(0,1) = y_t/y_t.norm();
    }
    else
    {
        y_t = init_contact_pose.rotation().block<3,1>(0,1);
        y_t = y_t - x_pp*(y_t.transpose()*x_pp);
        if (y_t.norm() > 1e-6)
        {
            projection_plane.block<3,1>(0,1) = y_t/y_t.norm();
        }
        else
        {
            projection_plane << 1., 0. , 0., 1., 0., 0.; 
        }
    }

    std::cout << CLASS_NAMESPACE << __func__ << " Computed plane is: \n" << projection_plane << std::endl;


    // Correct computation of on-plane projection matrix using pseudo inverse
    Eigen::Matrix<double,2,2> pTp = projection_plane.transpose()*projection_plane;
    Eigen::Matrix<double,3,3> proj_matrix = projection_plane*pTp.inverse()*projection_plane.transpose();
    Eigen::Vector3d x_projected_start = proj_matrix * x_i;
    //norm of x_projected
    double n_x_projected = x_projected_start.norm();
    if (n_x_projected <= 1e-3)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : sliding axis of starting pose orthogonal to sliding plane");
        response_.data = "error";
        return;
    }
    x_projected_start /= n_x_projected;
        
    Eigen::Vector3d x_projected_goal = proj_matrix * goal_pose_eigen.rotation().leftCols(1);
    
    double n_x_projected_goal = x_projected_goal.norm();
    if (n_x_projected_goal <= 1e-3)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : sliding axis of goal pose orthogonal to sliding plane");
        response_.data = "error";
        return;
    }
    x_projected_goal /= n_x_projected_goal;
    
#if DEBUG    
    std::cout << "Projection plane: " << std::endl << projection_plane << std::endl;
    std::cout << "x_projected_start: " << x_projected_start.transpose() << std::endl;
    std::cout << "x_projected_goal: " << x_projected_goal.transpose() << std::endl;
#endif
    
    BezierCurve::PointVector point_for_planning;
    point_for_planning.resize(point_for_planning.RowsAtCompileTime, 4);

    // FOR TILTING
    // Choosing 2 direction vectors to define 2 aux control points to get circular arc bezier on computed plane
    // 1) first vector is z_source or -z_source (the one in direction of xpp)
    // 2) second vector is the component of xpp perpendicular to z_source on the computed plane
    Eigen::Vector3d z_source;
    tf::vectorKDLToEigen(source_frame_tmp.M.UnitZ(), z_source);
    Eigen::Vector3d _z_source = -z_source;
    Eigen::Vector3d control1;
    Eigen::Vector3d control2;

    double dotzxpp = x_pp.transpose()*z_source;
    double dot_zxpp = x_pp.transpose()*_z_source;

    if(dotzxpp > dot_zxpp){
        control1 = dotzxpp*z_source;
        control2 = control1 - x_pp;
        control1 = control1/control1.norm();
        control2 = control2/control2.norm();
    }
    else if(dotzxpp < dot_zxpp){
        control1 = dot_zxpp*_z_source;
        control2 = control1 - x_pp;
        control1 = control1/control1.norm();
        control2 = control2/control2.norm();
    }
    else{
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " These poses are not for tilting. Some error occured!!!");
    }

#if DEBUG
    std::cout << CLASS_NAMESPACE << __func__ << " The two control vectors for tilting are: " << control1 << " and " << control2 << std::endl;
#endif
    //--------------

    BezierCurve::PointVector aux_point_1;
    BezierCurve::PointVector aux_point_2;

    // Choosing correct aux control points based on current transition: TILT or SLIDE
    if(req.current_transition_tilting){
        double k_tmp = 0.55228;
        Eigen::Vector3d d_pp = goal_pose_eigen.translation() - init_contact_pose.translation();
        double r_tmp = d_pp.norm()/SQRT2;
        // a point close to the init point
        aux_point_1 = init_contact_pose.translation() + control1*k_tmp*r_tmp;
        // a point close to the goal point
        aux_point_2 = goal_pose_eigen.translation() + control2*k_tmp*r_tmp;

        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << "TRYING TO TILT!!!");
    }
    else{
        // a point close to the init point
        aux_point_1 = init_contact_pose.translation() + x_projected_start*fixed_translation_bezier;
        // a point close to the goal point
        aux_point_2 = goal_pose_eigen.translation() - x_projected_goal*fixed_translation_bezier;
    }
       

    point_for_planning << init_contact_pose.translation(), aux_point_1 , aux_point_2, goal_pose_eigen.translation();

    planner_bezier_curve.init_curve(point_for_planning);

    std::cout << "The projection plane is: " << std::endl << projection_plane << std::endl;
    std::cout << CLASS_NAMESPACE << __func__ << " The start and end vectors for tilting are: " << init_contact_pose.translation() << " and " << goal_pose_eigen.translation() << std::endl;
    std::cout << CLASS_NAMESPACE << __func__ << " The two control vectors for tilting are: " << control1 << " and " << control2 << std::endl;

#if DEBUG
    char temp_char; std::cin >> temp_char;
#endif

    int num_samples = 20;
    
    std::vector <geometry_msgs::Pose > waypoints;
    std::vector <geometry_msgs::Pose > waypoints_tmp;
    Eigen::Affine3d contact_init_rotation;
    for(double s_bezier = 0.0; s_bezier<=1.0; s_bezier+=1.0/num_samples)
    {
        BezierCurve::Point res = planner_bezier_curve.compute_point(s_bezier);
        Eigen::Quaterniond rot;
        BezierCurve::Point dres = planner_bezier_curve.compute_derivative(s_bezier);  
        compute_orientation_from_vector(dres, rot);
        Eigen::Affine3d eigen_waypoint = Eigen::Translation3d(res)*rot;
        if(s_bezier == 0.0)
            contact_init_rotation = eigen_waypoint.inverse()*world_ee*ee_contact;
        geometry_msgs::Pose newWaypoint;
        tf::poseEigenToMsg(eigen_waypoint*contact_init_rotation, newWaypoint);
        waypoints.push_back(newWaypoint);
#if DEBUG_VISUAL
        tf::poseEigenToMsg(eigen_waypoint, newWaypoint);
        waypoints_tmp.push_back(newWaypoint);
#endif
    }
    
    moveit_msgs::RobotTrajectory planned_joint_trajectory;
    std::vector<double> single_distances({0.5,0.5,0.5,1.0,2.0,2.0,2.0});
    
    KDL::Frame ee_contact_kdl;
    tf::transformEigenToKDL(ee_contact, ee_contact_kdl);
    double completed;
    {
        auto ik_check_ = sikm.getIkCheckReadyForPlanning();
        ik_check_->getChainAndSolvers(req.ee_name)->changeTip(ee_contact_kdl);
        ik_check_->getChainAndSolvers(req.ee_name)->initSolvers();
        ik_check_->reset_robot_state(rs);
        
        uint trials_nr(100), attempts_nr(100);
        bool check_collisions(false);
        double allowed_distance(2.5);
        
        completed = ik_check_->computeTrajectoryFromWPs(planned_joint_trajectory,waypoints,req.ee_name,check_collisions,allowed_distance,single_distances, trials_nr, attempts_nr);
        
#if DEBUG_VISUAL
        publushStuff(waypoints);
        publushStuff(waypoints_tmp);
        visual_tools_->publishSphere(init_contact_pose.translation(), rviz_visual_tools::BLUE, rviz_visual_tools::scales::LARGE);
        visual_tools_->publishSphere(aux_point_1, rviz_visual_tools::BLUE, rviz_visual_tools::scales::LARGE);
        visual_tools_->publishSphere(aux_point_2, rviz_visual_tools::BLUE, rviz_visual_tools::scales::LARGE);
        visual_tools_->publishSphere(goal_pose_eigen.translation(), rviz_visual_tools::BLUE, rviz_visual_tools::scales::LARGE);
#endif
        
        if(completed != 1.0)
        {
            ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to get trajectory from waypoints, trying again reducing the weight on x- and y-axis tasks...");
        
            std::cout << "Recomputing IK using different weights!" << std::endl;
            char y;
            Eigen::Matrix<double,6,1> Wx;
            Wx.setOnes();
            Wx(3) = 0.0;
            Wx(4) = 0.0;
            planned_joint_trajectory.joint_trajectory.points.clear();
            ik_check_->getChainAndSolvers(req.ee_name)->changeIkTaskWeigth(Wx,true);
            ik_check_->reset_robot_state(rs);
            completed = ik_check_->computeTrajectoryFromWPs(planned_joint_trajectory,waypoints,req.ee_name,check_collisions,allowed_distance,single_distances, trials_nr, attempts_nr);
            
            Wx.setOnes();
            ik_check_->getChainAndSolvers(req.ee_name)->changeIkTaskWeigth(Wx,false);
        }
        
        ik_check_->getChainAndSolvers(req.ee_name)->changeTip(KDL::Frame::Identity());
        ik_check_->getChainAndSolvers(req.ee_name)->initSolvers();
    }
    if(completed != 1.0)
    {
        double tmp_roll, tmp_pitch, tmp_yaw;
        Object_PreSlide.M.GetEulerZYX(tmp_roll, tmp_pitch, tmp_yaw);
        std::cout << "Current Object_PreSlide is: " << "Z: " << tmp_roll << " Y: " << tmp_pitch << " X: " << tmp_yaw << std::endl;
        Object_PreTilt.M.GetEulerZYX(tmp_roll, tmp_pitch, tmp_yaw);
        std::cout << "Current Object_PreTilt is: " << "Z: " << tmp_roll << " Y: " << tmp_pitch << " X: " << tmp_yaw << std::endl;
        if(req.current_transition_tilting){
            std::cout << "current_transition_tilting is TRUE" << std::endl;
        }
        else{
            std::cout << "current_transition_tilting is FALSE" << std::endl;
        }

        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to get trajectory from waypoints, returning");
        response_.data = "error";
        return;
    }
    
    // removing waypoints at 0 distance in time from the start
    while(planned_joint_trajectory.joint_trajectory.points.begin()->time_from_start == ros::Duration(0.0))
        planned_joint_trajectory.joint_trajectory.points.erase(planned_joint_trajectory.joint_trajectory.points.begin());
    sikm.resetPlanningRobotState(group_name, planned_joint_trajectory);
    sikm.swapTrajectory(req.ee_name, planned_joint_trajectory);
    
    dual_manipulation_shared::scene_object_service::Request req_obj;
    req_obj.command = dual_manipulation::ik_control::ADD_OBJECT;
    req_obj.object_id = req.attObject.object.id;
    req_obj.attObject.object.header.frame_id = "world";
    req_obj.attObject.object.id = req.attObject.object.id;
    req_obj.attObject.object.mesh_poses.resize(1);
    
    KDL::Frame goal_pose_kdl;
    tf::poseMsgToKDL(goal_pose, goal_pose_kdl);

    //Choosing Object_Tilt or Object_Slide appropriately
    if(req.current_transition_tilting){
        tf::poseKDLToMsg(goal_pose_kdl*Object_Tilt.Inverse(), req_obj.attObject.object.mesh_poses.at(0));
    }
    else{
        tf::poseKDLToMsg(goal_pose_kdl*Object_Slide.Inverse(), req_obj.attObject.object.mesh_poses.at(0));
    }

    req_obj.object_db_id = req.object_db_id;
    
    sikm.sceneObjectManager->manage_object(req_obj);
    
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

void SlidingCapability::add_target(const dual_manipulation_shared::ik_service::Request& req)
{
    if (req.ee_pose.empty()){
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to set target for sliding, returning");
        response_.data = "error";
        return;
    }
    
    targets_[req.ee_name] = req.ee_pose.at(0);    
}



