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
    error_state processHeartbeats();
    void assess();

private:
  
    SM_LiDAR^ SM_LiDAR_;
    SM_VehicleControl^ SM_VC_;

    Stopwatch^ watch_;
    array<double>^ laserX_;
    array<double>^ laserY_;
    double speed_;
    double steering_;
    double maxSpeed_;
    bool steerLimited_;
    double steerOut_;
    int state_;
   

    
};
