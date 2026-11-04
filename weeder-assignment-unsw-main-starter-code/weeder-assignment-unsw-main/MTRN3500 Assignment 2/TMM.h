#pragma once

#include <UGVModule.h>
#include "SMObjects.h"
#include "Lidar.h"
#include "GNSS.h"

#using <System.dll>
#include <cstdint>
using namespace System;
using namespace System::Threading;
using namespace System::Diagnostics;

ref class ThreadManagement : public UGVModule {
public:
    // Create shared memory objects
    error_state setupSharedMemory();

    // Send/Recieve data from shared memory structures
    error_state processSharedMemory() override;

    // Shutdown all modules in the software
    void shutdownModules();

    // Get Shutdown signal for module, from Thread Management SM
    bool getShutdownFlag() override;

    // Thread function for TMM
    void threadFunction() override;


private:
   
   
    // Add any additional data members or helper functions here
    ref struct ThreadProperties
    {
        ThreadStart^ ThreadStart_;
        bool Critical;
        String^ ThreadName;
        uint8_t BitID;

        ThreadProperties(ThreadStart^ start, bool critical, uint8_t bit_id, String^ threadname)
        {
            ThreadStart_ = start;
            Critical = critical;
            BitID = bit_id;
            ThreadName = threadname;
        }
    };
    SM_ThreadManagement^ TMT_SM_TM_;
    SM_LiDAR^ TMT_SM_LiDAR_;
    SM_GNSS^ TMT_SM_GNSS_;
    array<Stopwatch^>^ TMT_StopwatchList;
    array<Thread^>^ ThreadList;
    array<ThreadProperties^>^ ThreadPropertiesList;
    error_state processHeartbeats();
    
};
