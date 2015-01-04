#ifndef DUAL_MANIPULATION_TRAJECTORY_GENERATOR
#define DUAL_MANIPULATION_TRAJECTORY_GENERATOR

#include <ros/ros.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames.hpp>

namespace dual_manipulation
{
    namespace ik_control
    {  
    /**
      * @brief A three terms polynomial
      * 
      */
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

    /**
     * @brief A class for the line trajectory parameters
     * 
     */
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

    /**
     * @brief The trajectory generator class
     * 
     */
    class trajectory_generator
    {
    public:
	trajectory_generator();
	~trajectory_generator();
	
	bool line_trajectory(double t, KDL::Frame& desired_pose, KDL::Twist& desired_velocity);

	/**
	 * @brief To set the line trajectory parameters.
	 * 
	 * @param time_ The time for which I want to compute the next pose.
	 * @param initial_pose_ Starting pose.
	 * @param final_pose_ Final pose, will be used to compute the displacement (\p final_pose_  - \p initial_pose_ ).
	 * @return bool
	 */
	bool initialize_line_trajectory(double time_, const KDL::Frame& initial_pose_, const KDL::Frame& final_pose_);
	
    private:
	/**
	* @brief This is the function to interpolate two poses.
	* Note that this approach use the displacement pose as \p final_pose.
	* 
	* @param poly The polynomial of the interpolation.
	* @param final_pose The final desired pose.
	* @param time The end effector pose will be computed for time \p time.
	* @param t_f The final time (at time \p t_f the end effector will be in the \p final_pose ).
	* @return KDL::Vector the pose at time \p time.
	*/
	KDL::Vector interpolation(const polynomial& poly, const KDL::Vector& final_pose, double time, double t_f);
	
	polynomial line_polynomial;
	line_parameters line_param;
    };
    }
}
#endif //DUAL_MANIPULATION_TRAJECTORY_GENERATOR