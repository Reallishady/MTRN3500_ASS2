#using <System.dll>
#include "GNSS.h"

using namespace System;
using namespace System::Threading;
using namespace System::Diagnostics;

GNSS::GNSS(SM_ThreadManagement^ SM_TM, SM_GNSS^ SM_GNSS)
{
	GNSS_SM_TM_ = SM_TM;
	GNSS_SM_GNSS_ = SM_GNSS;
	GNSS_Stopwatch = gcnew Stopwatch;
}

error_state GNSS::setupShareMemory()
{
	return error_state::SUCCESS;
}

void GNSS::threadFunction()
{
	//int i = 0;
	Console::WriteLine("GNSS Thread is starting");

	//wait for the barrier
	GNSS_SM_TM_->ThreadBarrier->SignalAndWait();

	//start the stopwatch
	GNSS_Stopwatch->Start();

	//start the thread loop
	while (!getShutdownFlag())
	{
		Console::WriteLine("GNSS Thread is running");
		processHeartbeats();
		if (communicate() == SUCCESS && checkData() == SUCCESS)
		{
			processSharedMemory();//process the data
		}
		Thread::Sleep(20);

		//if (i++ > 50) break;
	}

	Console::WriteLine("GNSS Thread is terminated");
}

error_state GNSS::processHeartbeats()
{
	//check if the GNSS bit in the hearbeat byte is low
	if ((GNSS_SM_TM_->heartbeat & bit_GNSS) == 0)
	{
		//put the GNSS bit high
		GNSS_SM_TM_->heartbeat |= bit_GNSS;
		//reset the stopwatch
		GNSS_Stopwatch->Restart();
	}

	else
	{
		//check if the time elasped exceed the crash time limit
		if (GNSS_Stopwatch->ElapsedMilliseconds > CRASH_LIMIT)
		{
			//shut down all threads
			shutdownThreads();
			return error_state::ERR_TMT_FAILURE;
		}
	}

	return error_state::SUCCESS;
}

void GNSS::shutdownThreads()
{
	GNSS_SM_TM_->shutdown = 0xFF;
}
bool GNSS::getShutdownFlag()
{
	return GNSS_SM_TM_->shutdown & bit_GNSS;
}

error_state GNSS::communicate()
{
	return error_state::SUCCESS;
}

error_state GNSS::checkData()
{
	return error_state::SUCCESS;
}

error_state GNSS::processSharedMemory()
{
	return error_state::SUCCESS;
}
error_state GNSS::connect(String^ hostName, int portNumber) {
	return error_state::SUCCESS;
}
