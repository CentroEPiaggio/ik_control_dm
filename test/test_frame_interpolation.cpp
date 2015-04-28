#include <ros/ros.h>
#include <iostream>

#define BIMANUAL 1

#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>

void poseCallback(const tf::Transform& transform, std::string name)
{
  static tf::TransformBroadcaster br;

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
  
  // geometry_msgs::Pose msg;
  // tf::poseTFToMsg(transform,msg);
  // ROS_INFO_STREAM("Sent transform " << name << "\n" << msg);
}

void pub_frames()
{
  static tf::TransformBroadcaster br;

  KDL::Frame start(KDL::Frame::Identity());
  KDL::Frame target;
  tf::Transform ee_pose;
  
  target.p = KDL::Vector(-0.6,-0.1,0.5);
  target.M = KDL::Rotation::Rot(KDL::Vector(1.0,0.0,1.0),M_PI);
  
  tf::Transform transform;
  tf::poseKDLToTF(start,transform);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/vito_anchor", "start"));
  tf::poseKDLToTF(target,transform);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/vito_anchor", "target"));
  
  int steps = 10;
  KDL::Twist tw;
  tw = KDL::diff(start,target);
  
  for(int i=0; i<=steps; i++)
  {
    KDL::Frame tmp = KDL::addDelta(start,tw,(1.0/steps)*i);
    tf::poseKDLToTF(tmp,ee_pose);
    poseCallback(ee_pose, "pose" + std::to_string(i));
  }
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> test_ik_check_capability "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_ik_check_capability");
    
    ros::NodeHandle n;
    
    ros::AsyncSpinner sp(1);
    sp.start();
    
    char y;
    std::cout << "You will need to KILL this program to exit. Press any key to continue... ";
    std::cin >> y;
    
    while(n.ok())
    {
      pub_frames();
      usleep(200000);
    }
    
    return 0;
}