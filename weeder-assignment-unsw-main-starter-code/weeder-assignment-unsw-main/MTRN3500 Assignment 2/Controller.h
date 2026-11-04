#pragma once
#include "UGVModule.h"
#include "SMObjects.h"
#include "ControllerInterface.h"



ref class Controller : public UGVModule
{
public:
    Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC);
    ~Controller();

    virtual void threadFunction() override;
    virtual error_state processSharedMemory() override;
    virtual bool getShutdownFlag() override;

private:
    SM_VehicleControl^ SM_VC_;
    ControllerInterface* controller_;

    Stopwatch^ Controller_Stopwatch;
};
