#include "CrashAvoidance.h"

CrashAvoidance::CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_LiDAR^ SM_lidar, SM_VehicleControl^ SM_VC)
{
    SM_TM_ = SM_TM;
    SM_LiDAR_ = SM_lidar;
    SM_VC_ = SM_VC;
    watch_ = gcnew Stopwatch;
}

CrashAvoidance::~CrashAvoidance()
{
}

void CrashAvoidance::threadFunction()
{
    SM_TM_->ThreadBarrier->SignalAndWait();

    while (!getShutdownFlag())
    {
        SM_TM_->heartbeat |= bit_CRASHAVOIDANCE;   
        System::Threading::Thread::Sleep(20);
    }

    System::Console::WriteLine("CrashAvoidance thread terminated");
}

error_state CrashAvoidance::processSharedMemory()
{
    return SUCCESS;
}

bool CrashAvoidance::getShutdownFlag()
{
    return (SM_TM_->shutdown & bit_CRASHAVOIDANCE) != 0;
}

