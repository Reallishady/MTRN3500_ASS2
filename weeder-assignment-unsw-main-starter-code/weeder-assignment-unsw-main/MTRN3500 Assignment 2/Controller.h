#pragma once
#include "ControllerInterface.h"
#include "UGVModule.h"
#include "SMObjects.h"



ref class Controller : public UGVModule
{
public:
    Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC);
    ~Controller();

    virtual void threadFunction() override;
    virtual error_state processSharedMemory() override;
    virtual bool getShutdownFlag() override;

    error_state processHeartbeats();

private:
    SM_VehicleControl^ SM_VC_;
    ControllerInterface* controller_;

    Stopwatch^ Controller_Stopwatch;

    double speed;
    double steering;
};
