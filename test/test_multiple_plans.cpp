#include <dual_manipulation_ik_control/move_group_interface.h>
#include <thread>

#define NUM_THREADS 2
#define NUM_RRTStar 1

std::vector<dual_manipulation::ik_control::MoveGroup*> moveGroups;
std::vector<dual_manipulation::ik_control::MotionPlan> plan(NUM_THREADS);
std::vector<double> path_length(NUM_THREADS,0.0);

void planning_thread(int i,double* path_length)
{
  ROS_INFO_STREAM("Starting thread #" << i);
  moveGroups.at(i)->plan(plan.at(i));
  for(auto point:plan.at(i).trajectory_.joint_trajectory.points)
    for(auto q:point.positions)
      *path_length+=std::abs(q);
  ROS_INFO_STREAM("Stopping thread #" << i << " | path_length=" << *path_length);
}

int main(int argc, char **argv)
{
  std::cout<<std::endl;
  std::cout<<"|Dual manipulation| -> test_multiple_plans "<<std::endl;
  std::cout<<std::endl;

  ros::init(argc, argv, "test_multiple_plans");
  ros::NodeHandle n;
  
  ros::AsyncSpinner as(1);
  as.start();
  
  for(int i=0; i<NUM_THREADS; i++)
    moveGroups.push_back(new dual_manipulation::ik_control::MoveGroup("left_hand_arm"));
  
  geometry_msgs::Pose ee_pose;
  ee_pose.position.x = -0.5;
  ee_pose.position.y = -0.3;
  ee_pose.position.z = 0.5;
  ee_pose.orientation.w = 1.0;
  
  for(int i=0; i<NUM_THREADS; i++)
  {
    if(i<NUM_RRTStar)
      moveGroups.at(i)->setPlannerId("RRTstarkConfigDefault");
    else
      moveGroups.at(i)->setPlannerId("RRTConnectkConfigDefault");
    moveGroups.at(i)->setNumPlanningAttempts(1000);
    moveGroups.at(i)->setPlanningTime(2);
    moveGroups.at(i)->setPoseTarget(ee_pose);
  }
  
  sleep(1);
  
  std::cout << "path_length : ";
  for(int i=0; i<path_length.size(); i++)
    std::cout << i << ":=" << path_length.at(i) << " ";
  std::cout << std::endl;
  
  for(int i=0; i<NUM_THREADS; i++)
  {
    new std::thread(boost::bind(&planning_thread,i,&path_length.at(i)));
    usleep(1000);
  }
  
  char y;
  std::cout << "Press any key to exit (you may want to wait for planning_threads to stop!)..." << std::endl;
  std::cin >> y;
  
  std::cout << "path_length : ";
  for(int i=0; i<path_length.size(); i++)
    std::cout << i << ":=" << path_length.at(i) << " ";
  std::cout << std::endl;
  
  return 0;
}


