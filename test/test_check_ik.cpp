#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "dual_manipulation_shared/ik_service.h"
#include "visualization_msgs/Marker.h"

std::vector<visualization_msgs::Marker> markers;

visualization_msgs::Marker left_marker;
visualization_msgs::Marker right_marker;

void callback_l(const std_msgs::String::ConstPtr& str)
{    
    if(str->data =="done")
    {
	markers.push_back(left_marker);
	ROS_INFO_STREAM(left_marker.ns<<" "<<left_marker.id<<" "<<str->data);
	left_marker.id += 1;
    }
}

void callback_r(const std_msgs::String::ConstPtr& str)
{    
    if(str->data =="done")
    {
        markers.push_back(right_marker);
	ROS_INFO_STREAM(right_marker.ns<<" "<<right_marker.id<<" "<<str->data);
	right_marker.id += 1;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_check_ik");
    
    left_marker.color.a=1;
    left_marker.color.r=0;
    left_marker.color.g=1;
    left_marker.color.b=0;
    left_marker.ns = "left_hand";
    left_marker.type = visualization_msgs::Marker::SPHERE;
    left_marker.id = 0;
    left_marker.header.frame_id = "world";
    left_marker.scale.x = 0.05;
    left_marker.scale.y = 0.05;
    left_marker.scale.z = 0.05;

    right_marker.color.a=1;
    right_marker.color.r=1;
    right_marker.color.g=0;
    right_marker.color.b=0;
    right_marker.ns = "right_hand";
    right_marker.type = visualization_msgs::Marker::SPHERE;
    right_marker.id = 0;
    right_marker.header.frame_id = "world";
    right_marker.scale.x = 0.05;
    right_marker.scale.y = 0.05;
    right_marker.scale.z = 0.05;
    
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> test_check_ik "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_check_ik");
    
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    ros::Subscriber lsub = n.subscribe("/ik_control/left_hand/check_done",1,callback_l);
    ros::Subscriber rsub = n.subscribe("/ik_control/right_hand/check_done",1,callback_r);
    ros::Publisher pub = n.advertise<visualization_msgs::Marker>( "/test/marker", 1000 );
    dual_manipulation_shared::ik_service srv;
    
    srv.request.command = "ik_check"; //"plan"; "ik_check";
    srv.request.ee_name = "right_hand";

    geometry_msgs::Pose temp_pose;
    
    temp_pose.orientation.w=1;
    temp_pose.orientation.x=0;
    temp_pose.orientation.y=0;
    temp_pose.orientation.z=0;
    
    for(double x=-1;x<=1;x+=0.3)
	for(double y=-1.1;y<=0;y+=0.3)
	    for(double z=0;z<=0.75;z+=0.3)
	    {
		temp_pose.position.x = x;
		temp_pose.position.y = y;
		temp_pose.position.z = z;
		
		srv.request.ee_pose.clear();
		srv.request.ee_pose.push_back(temp_pose);

		while (!client.call(srv))
		{
		    right_marker.pose = temp_pose;
		    ros::spinOnce();
		}
	    }

    srv.request.ee_name = "left_hand";
	    
    for(double x=-1;x<=1;x+=0.3)
	for(double y=1.1;y>=0;y-=0.3)
	    for(double z=0;z<=0.75;z+=0.3)
	    {		
		temp_pose.position.x = x;
		temp_pose.position.y = y;
		temp_pose.position.z = z;
		
		srv.request.ee_pose.clear();
		srv.request.ee_pose.push_back(temp_pose);

		while (!client.call(srv))
		{
		    left_marker.pose = temp_pose;
		    ros::spinOnce();
		}
	    }
    
    for(auto item:markers) pub.publish(item);
	    
    ros::spinOnce();

    return 0;
}