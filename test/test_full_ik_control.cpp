#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/ik_response.h"
#include "dual_manipulation_shared/scene_object_service.h"
#include <atomic>
#include "dual_manipulation_shared/ik_control_capabilities.h"

volatile std::atomic_bool doing_stuff(false);
typedef const dual_manipulation_shared::ik_responseConstPtr& msg_type;

void plan_callback(msg_type str)
{
    std::string type("Plan");
    ROS_INFO("%s for group %s (seq=%i) : %s",type.c_str(),str->group_name.c_str(),str->seq,str->data.c_str());
    doing_stuff.store(false);
}

void exec_callback(msg_type str) //, std::string type = std::string("NO type"))
{
    std::string type("Exec/Grasp/Ungrasp");
    ROS_INFO("%s for group %s (seq=%i) : %s",type.c_str(),str->group_name.c_str(),str->seq,str->data.c_str());
    doing_stuff.store(false);
}

void service_ret_print(const std::string& prefix, const std::string& cmd, bool res)
{
    if(!res)
    {
        ROS_ERROR_STREAM(prefix << "Service \'" << cmd << "\' rejected!");
        abort();
    }
    else
    {
        ROS_INFO_STREAM(prefix << "Service \'" << cmd << "\' accepted!");
    }
    
    while(doing_stuff.load() && ros::ok())
        usleep(5000);
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> full_ik_control_test "<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Test the major functionalities in dual_manipulation_ik_control. In order to do so, you have to run:"<<std::endl;
    std::cout<<"1: roslaunch vito_moveit_configuration demo.launch"<<std::endl;
    std::cout<<"2: rosrun dual_manipulation_ik_control dual_manipulation_ik_control"<<std::endl;
    std::cout<<"3: rosrun dual_manipulation_ik_control full_ik_control_test"<<std::endl;
    std::cout<<std::endl;
    
    ros::init(argc, argv, "full_ik_control_test");
    
    ros::NodeHandle n;
    ros::AsyncSpinner aspin(1);
    aspin.start();
    ros::ServiceClient client_obj = n.serviceClient<dual_manipulation_shared::scene_object_service>("scene_object_ros_service");
    ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    
    ros::Subscriber plan_sub = n.subscribe<msg_type>("ik_control/planning_done",1,plan_callback);
    ros::Subscriber exe_sub = n.subscribe<msg_type>("ik_control/action_done",1,exec_callback);
    ros::Subscriber grasp_sub = n.subscribe<msg_type>("ik_control/grasp_done",1,exec_callback);
    ros::Subscriber ungrasp_sub = n.subscribe<msg_type>("ik_control/ungrasp_done",1,exec_callback);
    // ros::Subscriber exe_sub = n.subscribe<msg_type>("ik_control/action_done",1,boost::bind(&exec_callback, _1, "Execution"));
    // ros::Subscriber grasp_sub = n.subscribe<msg_type>("ik_control/grasp_done",1,boost::bind(&exec_callback, _1, "Grasp"));
    // ros::Subscriber ungrasp_sub = n.subscribe<msg_type>("ik_control/ungrasp_done",1,boost::bind(&exec_callback, _1, "Ungrasp"));
    
    dual_manipulation_shared::ik_service srv;
    dual_manipulation_shared::scene_object_service srv_obj;
    
    if(!client_obj.waitForExistence())
        return -1;
    
    // Add object in the scene
    dual_manipulation_shared::scene_object_serviceRequest& ro(srv_obj.request);
    ro.command = "add";
    ro.object_id = "my_obj";
    ro.object_db_id = 2;
    ro.attObject.object.header.frame_id = "world";
    ro.attObject.object.id = "my_obj (object.id)";
    geometry_msgs::Pose pose;
    pose.position.x = -0.5;
    pose.orientation.w = 1.0;
    ro.attObject.object.mesh_poses.push_back(pose);
    
    service_ret_print("Object ",srv_obj.request.command,client_obj.call(srv_obj));
    
    // instantiate a variable which contains all needed names
    ik_control_capability ikc;
    
    // set a target for an end-effector
    dual_manipulation_shared::ik_serviceRequest& rs(srv.request);
    rs.command = ikc.name[ik_control_capabilities::SET_TARGET];
    rs.ee_name = "right_hand";
    pose.position.z = 0.5;
    pose.orientation.y = 0.5;
    pose.orientation.z = 0.5;
    pose.orientation.w = 0.707;
    rs.ee_pose.emplace_back(pose);
    doing_stuff.store(false);
    service_ret_print("IK ",srv.request.command,client.call(srv));
    
    // plan
    rs.command = ikc.name[ik_control_capabilities::PLAN];
    rs.ee_pose.clear();
    doing_stuff.store(true);
    rs.seq = 1;
    service_ret_print("IK ",srv.request.command,client.call(srv));
    
    // move
    rs.command = ikc.name[ik_control_capabilities::MOVE];
    doing_stuff.store(true);
    rs.seq++;
    service_ret_print("IK ",srv.request.command,client.call(srv));
    
    // grasp
    rs.command = ikc.name[ik_control_capabilities::GRASP];
    rs.grasp_trajectory.joint_names.push_back("right_hand_synergy_joint");
    trajectory_msgs::JointTrajectoryPoint gtp;
    gtp.positions.push_back(0.0);
    gtp.time_from_start = ros::Duration(0.5);
    rs.grasp_trajectory.points.push_back(gtp);
    gtp.positions.clear();
    gtp.positions.push_back(1.0);
    gtp.time_from_start = ros::Duration(1.0);
    rs.grasp_trajectory.points.push_back(gtp);
    rs.ee_pose.push_back(pose);
    pose.position.z -= 0.1;
    rs.ee_pose.push_back(pose);
    rs.object_db_id = ro.object_db_id;
    rs.attObject = ro.attObject;
    rs.attObject.link_name = "right_hand_palm_link";
    rs.attObject.object.header.frame_id = "right_hand_palm_link";
    rs.attObject.object.mesh_poses.at(0).position.x = 0.07;
    rs.attObject.object.mesh_poses.at(0).position.z = 0.05;
    doing_stuff.store(true);
    rs.seq++;
    service_ret_print("IK ",srv.request.command,client.call(srv));
    
    // needed to wait for the reset of internal ik_control variables...
    // TODO: remove this need in ik_control
    sleep(1);
    
    // ungrasp
    rs.command = ikc.name[ik_control_capabilities::UNGRASP];
    std::reverse(rs.ee_pose.begin(),srv.request.ee_pose.end());
    std::reverse(rs.grasp_trajectory.points.begin(),srv.request.grasp_trajectory.points.end());
    doing_stuff.store(true);
    rs.seq++;
    service_ret_print("IK ",srv.request.command,client.call(srv));
    
    // set home target
    rs.command = ikc.name[ik_control_capabilities::SET_HOME_TARGET];
    doing_stuff.store(false);
    rs.seq++;
    service_ret_print("IK ",srv.request.command,client.call(srv));
    
    // plan
    rs.command = ikc.name[ik_control_capabilities::PLAN];
    rs.ee_pose.clear();
    doing_stuff.store(true);
    rs.seq++;
    service_ret_print("IK ",srv.request.command,client.call(srv));
    
    // move
    rs.command = ikc.name[ik_control_capabilities::MOVE];
    doing_stuff.store(true);
    rs.seq++;
    service_ret_print("IK ",srv.request.command,client.call(srv));
    
    // set a bimanual target, one end-effector at a time
    rs.command = ikc.name[ik_control_capabilities::SET_TARGET];
    rs.ee_name = "right_hand";
    rs.ee_pose.clear();
    rs.ee_pose.emplace_back(pose);
    doing_stuff.store(false);
    service_ret_print("IK ",srv.request.command,client.call(srv));
    rs.ee_name = "left_hand";
    pose.position.y = -0.5;
    pose.position.x -= 0.2;
    rs.ee_pose.clear();
    rs.ee_pose.emplace_back(pose);
    doing_stuff.store(false);
    service_ret_print("IK ",srv.request.command,client.call(srv));
    
    // plan
    rs.command = ikc.name[ik_control_capabilities::PLAN_BEST_EFFORT];
    rs.ee_name = "full_robot";
    rs.ee_pose.clear();
    doing_stuff.store(true);
    rs.seq++;
    service_ret_print("IK ",srv.request.command,client.call(srv));
    
    // move
    rs.command = ikc.name[ik_control_capabilities::MOVE];
    doing_stuff.store(true);
    rs.seq++;
    service_ret_print("IK ",srv.request.command,client.call(srv));
    
    // home
    rs.command = ikc.name[ik_control_capabilities::HOME];
    doing_stuff.store(true);
    rs.seq++;
    service_ret_print("IK ",srv.request.command,client.call(srv));

    for(int i=0; i<3; i++)
    {
        ROS_INFO_STREAM("Finishing #" << i << "...");
        ros::spinOnce();
        usleep(500000);
    }
    
    return 0;
}