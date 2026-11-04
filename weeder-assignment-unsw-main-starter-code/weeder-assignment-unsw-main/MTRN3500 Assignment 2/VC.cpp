#include "VC.h"

VehicleControl::VehicleControl(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC)
{
    SM_TM_ = SM_TM;
    SM_VC_ = SM_VC;
    
    Stopwatch^ VC_Stopwatch = gcnew Stopwatch;
}

VehicleControl::~VehicleControl()
{
    if (Stream != nullptr) Stream->Close();
    if (Client != nullptr) Client->Close();
}

void VehicleControl::threadFunction()
{
    SM_TM_->ThreadBarrier->SignalAndWait();

    while (!getShutdownFlag())
    {
        SM_TM_->heartbeat |= bit_VC;   // rename to match SMObjects.h
        System::Threading::Thread::Sleep(20);
    }

    System::Console::WriteLine("VehicleControl thread terminated");
}

error_state VehicleControl::processSharedMemory()
{
    return SUCCESS;
}

bool VehicleControl::getShutdownFlag()
{
    return (SM_TM_->shutdown & bit_VC) != 0;
}

error_state VehicleControl::connect(System::String^ hostName, int portNumber)
{
    return SUCCESS;
}

error_state VehicleControl::communicate()
{
    return SUCCESS;
}