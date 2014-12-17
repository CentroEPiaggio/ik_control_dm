#include "trajectory_generator.h"
#include <complex>
#include <tf/tf.h>

using namespace dual_manipulation::ik_control;

trajectory_generator::trajectory_generator()
{

}

KDL::Vector trajectory_generator::interpolation(const polynomial& poly,const KDL::Vector& vec_f, double time, double t_f)
{
    return poly.a()*vec_f/(2.0*pow(t_f,3))*pow(time,3) + (poly.b()*vec_f)/(2.0*pow(t_f,4))*pow(time,4) + (poly.c()*vec_f)/(2.0*pow(t_f,5))*pow(time,5);
}

bool trajectory_generator::initialize_line_trajectory(double time, const KDL::Frame& start, const KDL::Frame& final)
{
    if(time<=0) return false;
    
    line_param.time = time;
    line_param.initial_pose = start;
    line_param.displacement.p = final.p-start.p;
    double r,p,y,r1,p1,y1;
    start.M.GetRPY(r,p,y);
    final.M.GetRPY(r1,p1,y1);
    line_param.displacement.M=KDL::Rotation::RPY(r1-r,p1-p,y1-y);
    line_param.initialized = true;
    return true;
}

bool trajectory_generator::line_trajectory(double t, KDL::Frame& pos_d, KDL::Twist& vel_d)
{
    if(!line_param.initialized) return false;
  
    polynomial poly,vel_poly;
    vel_poly.set_coefficients(60.0, -120.0, 60);
    
    KDL::Vector temp_vector_r, start_vector_r, dis_vector_r;
    
    KDL::Vector temp_vel_vector_p, temp_vel_vector_r;

    double ro,pi,ya;
    line_param.initial_pose.M.GetRPY(ro,pi,ya);
    start_vector_r.data[0] = ro;
    start_vector_r.data[1] = pi;
    start_vector_r.data[2] = ya;
        
    double ro_,pi_,ya_;
    line_param.displacement.M.GetRPY(ro_,pi_,ya_);
    dis_vector_r.data[0] = ro_;
    dis_vector_r.data[1] = pi_;
    dis_vector_r.data[2] = ya_;
    
    if(t >= 0.0 && t<=line_param.time)
    {
          pos_d.p = line_param.initial_pose.p + interpolation(poly,line_param.displacement.p,t,line_param.time);
	  temp_vector_r = start_vector_r + interpolation(poly,dis_vector_r,t,line_param.time);
	  
          temp_vel_vector_p = interpolation(vel_poly,line_param.displacement.p,t,line_param.time);
	  temp_vel_vector_r = interpolation(vel_poly,dis_vector_r,t,line_param.time);

	  pos_d.M = KDL::Rotation::RPY(temp_vector_r.data[0],temp_vector_r.data[1],temp_vector_r.data[2]);
	  
          vel_d.vel=temp_vel_vector_p;
	  vel_d.rot=temp_vel_vector_r;
    }
    else if (t > line_param.time)
    {
          pos_d.p = line_param.initial_pose.p + line_param.displacement.p;
	  pos_d.M = KDL::Rotation::RPY(start_vector_r.data[0] + dis_vector_r.data[0],start_vector_r.data[1] + dis_vector_r.data[1],start_vector_r.data[2] + dis_vector_r.data[2]);
	  
	  vel_d.vel= KDL::Vector::Zero();
	  vel_d.rot = KDL::Vector::Zero();
    }
    return true;
}

trajectory_generator::~trajectory_generator()
{

}