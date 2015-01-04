#ifndef IK_CONTROL_H
#define IK_CONTROL_H

#include "trajectory_generator.h"
#include "dual_manipulation_shared/ik_service.h"
#include <std_msgs/String.h>

namespace dual_manipulation
{
namespace ik_control
{
  
/**
  * @brief This is a class that is used from the ros_server to perform a desired ik using a dedicated thread (one for each end effector).
  * 
  */
class ikControl
{
public:
    ikControl();
    ~ikControl();
    
    bool perform_ik(dual_manipulation_shared::ik_service::Request &req);
    
private:
    std::map<std::string,dual_manipulation::ik_control::trajectory_generator*> trj_gen;
    std::map<std::string,bool> busy;
    ros::NodeHandle node;
    std::map<std::string,ros::Publisher> hand_pub;
    std_msgs::String msg;

    /**
     * @brief this is the thread body, trajectory generation and ik control are performed in it
     * 
     * @param req
     *   the same req from the @e ik_service
     * @return void
     */
    void ik_thread(dual_manipulation_shared::ik_service::Request req);
};

}
}

#endif // IK_CONTROL_H
