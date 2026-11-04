#include "Controller.h"

Controller::Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC)
{
    SM_TM_ = SM_TM;
    SM_VC_ = SM_VC;
    controller_ = nullptr;
    Stopwatch^ Controller_Stopwatch = gcnew Stopwatch;
}

Controller::~Controller()
{
    delete controller_;
}

void Controller::threadFunction()
{
    SM_TM_->ThreadBarrier->SignalAndWait();

    while (!getShutdownFlag())
    {
        SM_TM_->heartbeat |= bit_CONTROLLER;   
        System::Threading::Thread::Sleep(20);
    }

    System::Console::WriteLine("Controller thread terminated");
}

error_state Controller::processSharedMemory()
{
    return SUCCESS;
}

bool Controller::getShutdownFlag()
{
    return (SM_TM_->shutdown & bit_CONTROLLER) != 0;
}