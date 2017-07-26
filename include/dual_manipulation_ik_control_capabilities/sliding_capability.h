/*********************************************************************
*
* Software License Agreement (BSD License)
*
*
*  Copyright (c) 2016, Hamal Marino <hamal dot marino at gmail dot com>
*  Copyright (c) 2016, Manuel Bonilla
*  Copyright (c) 2017, George Jose Pollayil <gpollayil at gmail dot com>
*  Copyright (c) 2017, Mathew Jose Pollayil <mathewjosepollayil at gmail dot com>
*  Copyright (c) 2017, Centro di Ricerca "E. Piaggio", University of Pisa
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef SLIDING_CAPABILITY_H_
#define SLIDING_CAPABILITY_H_

#include <dual_manipulation_ik_control_capabilities/generic_planning_capability.h>
#include <atomic>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <bezier_curve/bezier_curve.h>

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
    KDL::Frame Object_PreTilt, Object_Tilt;

    // adding info about the two main table and edge grasp ids for sliding
    int sCbt = 401641; // side C bottom table
    int sCtt = 401644; // side C top table
    int sCbe = 401841; // side C bottom edge
    int sCte = 401844; // side C top edge
    
private:
    /**
     * @brief Sets Object_PreSlide and Object_Slide of the class according to best hand position for achieving good sliding
     *        using a dot product test
     * 
     * @param source_position a geometry_msg Pose with info on source position and orientation
     * @param target_position a geometry_msg Pose with info on target position and orientation
     * 
     * @return true if the two KDL Frames were set correctly, false otherwise
     */
    bool set_hand_pose_sliding(geometry_msgs::Pose source_position, geometry_msgs::Pose target_position, int current_source_grasp);
    
    /**
     * @brief Sets Object_PreTilt and Object_Tilt of the class according to best hand position for achieving good tilting
     *        using a dot product test (same as sliding)
     * 
     * @param source_position a geometry_msg Pose with info on source position and orientation
     * @param target_position a geometry_msg Pose with info on target position and orientation
     * 
     * @return true if the two KDL Frames were set correctly, false otherwise
     */
    bool set_hand_pose_tilting(geometry_msgs::Pose source_position, geometry_msgs::Pose target_position, int current_source_grasp);

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
