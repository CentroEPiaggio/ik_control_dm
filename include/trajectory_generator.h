#ifndef DUAL_MANIPULATION_TRAJECTORY_GENERATOR
#define DUAL_MANIPULATION_TRAJECTORY_GENERATOR

#include <ros/ros.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames.hpp>

namespace dual_manipulation
{
    namespace ik_control
    {  
    class polynomial
    {
    public:
	polynomial()
	{
	    a_=20.0;
	    b_=-30.0;
	    c_=12.0;
	}

	void set_coefficients(double a, double b, double c)
	{
	    a_=a;
	    b_=b;
	    c_=c;
	}
	
	double a() const{ return a_;}
	double b() const{ return b_;}
	double c() const{ return c_;}
	
    private:
	double a_,b_,c_;
    };

    class line_parameters
    {
    public:
	line_parameters()
	{
	    time=1;

	    initial_pose = KDL::Frame::Identity();

	    displacement = KDL::Frame::Identity();

	    initialized = false;
	}
	
	bool initialized;
	double time;
	KDL::Frame initial_pose;
	KDL::Frame displacement;
    };

    class trajectory_generator
    {
    public:
	trajectory_generator();
	~trajectory_generator();
	
	bool line_trajectory(double t, KDL::Frame& desired_pose, KDL::Twist& desired_velocity);
	bool initialize_line_trajectory(double time_, const KDL::Frame& initial_pose_, const KDL::Frame& final_pose_);
	
    private:
	KDL::Vector interpolation(const polynomial& poly, const KDL::Vector& final_pose, double time, double t_f);
	
	polynomial line_polynomial;
	line_parameters line_param;
    };
    }
}
#endif //DUAL_MANIPULATION_TRAJECTORY_GENERATOR