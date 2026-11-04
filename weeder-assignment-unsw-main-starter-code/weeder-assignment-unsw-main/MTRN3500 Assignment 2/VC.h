#pragma once

#include "NetworkedModule.h"
#include "SMObjects.h"



ref class VehicleControl : public NetworkedModule
{
public:
    VehicleControl(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC);
    ~VehicleControl();

    
    virtual void threadFunction() override;
    virtual error_state processSharedMemory() override;
    virtual bool getShutdownFlag() override;

    
    virtual error_state connect(System::String^ hostName, int portNumber) override;
    virtual error_state communicate() override;

private:
    SM_VehicleControl^ SM_VC_;
    Stopwatch^ VC_Stopwatch;
};