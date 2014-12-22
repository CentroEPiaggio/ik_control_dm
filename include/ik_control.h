#ifndef IK_CONTROL_H
#define IK_CONTROL_H

#include "trajectory_generator.h"
#include "dual_manipulation_shared/ik_service.h"
#include <std_msgs/String.h>

namespace dual_manipulation
{
namespace ik_control
{
  
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
    ros::Publisher pub;
    std_msgs::String msg;

    void ik_thread(dual_manipulation_shared::ik_service::Request req);
};

}
}

#endif // IK_CONTROL_H
