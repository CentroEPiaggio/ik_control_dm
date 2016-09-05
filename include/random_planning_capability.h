#include "abstract_capability.h"

namespace dual_manipulation
{
namespace ik_control
{

class randomPlanningCapability : public abstractCapability
{
public:
    randomPlanningCapability(shared_ik_memory& sikm_);
    virtual ~randomPlanningCapability();
    virtual bool isComplete() {return true;}
    virtual void performRequest(dual_manipulation_shared::ik_serviceRequest req);
    virtual bool canRun() {return true;}
    virtual bool canPerformCapability(const ik_control_capabilities& ik_capability);
    virtual void reset();
private:
    shared_ik_memory& sikm;
};

}
}