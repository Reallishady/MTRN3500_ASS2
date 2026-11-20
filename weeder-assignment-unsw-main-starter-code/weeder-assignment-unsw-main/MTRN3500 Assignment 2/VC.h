#pragma once

#include "NetworkedModule.h"
#include "SMObjects.h"

using namespace System;
using namespace System::Threading;
using namespace System::Diagnostics;
using namespace System::IO;
using namespace System::Globalization;

const int VC_PORT = 25000;

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
    error_state processHeartbeats();

private:
    SM_VehicleControl^ SM_VC_;
    Stopwatch^ VC_Stopwatch;
    void shutdownModules()
    {
        SM_TM_->shutdown = 0xFF;
    }
    double speed_;
    double steering_;
    int wdog_;
    void disconnect()
    {
        try
        {
            if (Stream != nullptr) Stream->Close();
            if (Client != nullptr) Client->Close();
        }
        catch (Exception^)
        {
        }
    }
};
