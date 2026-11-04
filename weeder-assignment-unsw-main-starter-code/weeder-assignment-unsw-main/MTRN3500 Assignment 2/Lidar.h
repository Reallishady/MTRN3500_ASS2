#pragma once
#using <System.dll>
#include "SMObjects.h"
#include "NetworkedModule.h"


using namespace System;
using namespace System::Threading;
using namespace System::Diagnostics;

ref class LiDAR : public  NetworkedModule

{
public:
	LiDAR(SM_ThreadManagement^ SM_TM, SM_LiDAR^ SM_lidar);

	error_state setupShareMemory();
	void threadFunction() override;
	error_state processHeartbeats();
	void shutdownThreads();
	bool getShutdownFlag()override;
	error_state communicate()override;
	error_state connect(String^ hostName, int portNumber) override;
	error_state checkData();
	error_state processSharedMemory()override;

private:
	SM_ThreadManagement^ LiDAR_SM_TM_;
	SM_LiDAR^ LiDAR_SM_LiDAR_;
	Stopwatch^ LiDAR_Stopwatch;
};