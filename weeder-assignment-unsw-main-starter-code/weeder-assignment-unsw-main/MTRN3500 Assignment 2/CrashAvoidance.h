#pragma once

#include "UGVModule.h"
#include "SMObjects.h"


ref class CrashAvoidance : public UGVModule
{
public:
    CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_LiDAR^ SM_LiDAR, SM_VehicleControl^ SM_VC);
    ~CrashAvoidance();

  
    virtual void threadFunction() override;
    virtual error_state processSharedMemory() override;
    virtual bool getShutdownFlag() override;

private:
  
    SM_LiDAR^ SM_LiDAR_;
    SM_VehicleControl^ SM_VC_;

    Stopwatch^ watch_;
};