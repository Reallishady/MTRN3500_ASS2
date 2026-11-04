#pragma once
#using <System.dll>
#include "SMObjects.h"
#include "NetworkedModule.h"

using namespace System;
using namespace System::Threading;
using namespace System::Diagnostics;

ref class GNSS : public NetworkedModule
{
public:
	GNSS(SM_ThreadManagement^ SM_TM, SM_GNSS^ SM_GNSS);

	error_state setupShareMemory();
	void threadFunction() override;
	error_state processHeartbeats();
	void shutdownThreads();
	bool getShutdownFlag() override;
	error_state communicate() override;
	error_state connect(String^ hostName, int portNumber) override;
	error_state checkData();
	error_state processSharedMemory() override;

private:
	SM_ThreadManagement^ GNSS_SM_TM_;
	SM_GNSS^ GNSS_SM_GNSS_;
	Stopwatch^ GNSS_Stopwatch;
};