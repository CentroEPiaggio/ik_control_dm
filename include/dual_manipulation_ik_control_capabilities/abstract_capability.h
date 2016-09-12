#ifndef ABSTRACT_CAPABILITY_H_
#define ABSTRACT_CAPABILITY_H_

#include <dual_manipulation_shared/ik_control_capabilities.h>
#include <dual_manipulation_shared/ik_service.h>
#include <dual_manipulation_shared/ik_response.h>
#include <dual_manipulation_ik_control/shared_ik_memory.h>

namespace dual_manipulation
{
namespace ik_control
{
/**
 * @brief An abstract class implementing a generic capability for use inside ik_control: plan, move, grasp, ...
 */
class abstractCapability
{
public:
    abstractCapability(){}
    virtual ~abstractCapability(){}
    virtual bool isComplete()=0;
    virtual void performRequest(dual_manipulation_shared::ik_serviceRequest req)=0;
    virtual bool getResults(dual_manipulation_shared::ik_response& res)=0;
    virtual bool canRun()=0;
    /// could be associated with a type coming from ik_control_capabilities.h
    virtual bool canPerformCapability(const ik_control_capabilities& ik_capability) const=0;
    virtual void reset(){}
};

}
}

#endif // ABSTRACT_CAPABILITY_H_

// // // #include "lockstrap.h"
// // // 
// // // // class User {
// // // //     class Data {
// // // //         int a;
// // // //         float b;
// // // //         LOCKSTRAP(Data, std::mutex, a,b);
// // // //     } d;
// // // //     std::vector<long> x;
// // // //     // ...
// // // // };