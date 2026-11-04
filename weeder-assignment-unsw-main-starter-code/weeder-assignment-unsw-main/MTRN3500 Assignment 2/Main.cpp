// Controller.h first so windows.h (via ControllerInterface) gets included before
// any "using namespace System", otherwise IServiceProvider becomes ambiguous
#include "Controller.h"
#include "VC.h"
#include "CrashAvoidance.h"

#include "GNSS.h"
#include "Lidar.h"
#include "TMM.h"     // rename any of these to match your actual header names

using namespace System;
using namespace System::Threading;
using namespace System::Diagnostics;

// main stands in for the TM thread here so LiDAR can be tested on its own
int main()
{
	SM_ThreadManagement^ SM_TM = gcnew SM_ThreadManagement();
	SM_LiDAR^ SM_Lidar = gcnew SM_LiDAR();

	SM_TM->ThreadBarrier = gcnew Barrier(2);   // main + LiDAR
	SM_TM->shutdown = 0;
	SM_TM->heartbeat = 0;

	LiDAR^ lidar = gcnew LiDAR(SM_TM, SM_Lidar);
	Thread^ lidarThread = gcnew Thread(gcnew ThreadStart(lidar, &LiDAR::threadFunction));
	lidarThread->Start();

	SM_TM->ThreadBarrier->SignalAndWait();
	Console::WriteLine("Running, press q to quit");

	Stopwatch^ lidarWatch = Stopwatch::StartNew();
	bool quitPressed = false;

	while (lidarThread->IsAlive)
	{
		if (Console::KeyAvailable && Console::ReadKey(true).Key == ConsoleKey::Q)
		{
			Console::WriteLine("q pressed, shutting down");
			SM_TM->shutdown = 0xFF;
			quitPressed = true;
			break;
		}

		// same job as TM: clear LiDAR's bit, and notice if it stops setting it
		if (SM_TM->heartbeat & bit_LiDAR)
		{
			SM_TM->heartbeat &= ~bit_LiDAR;
			lidarWatch->Restart();
		}
		else if (lidarWatch->ElapsedMilliseconds > CRASH_LIMIT)
		{
			Console::WriteLine("LiDAR heartbeat lost, shutting down");
			SM_TM->shutdown = 0xFF;
			break;
		}

		Thread::Sleep(50);
	}

	if (!lidarThread->Join(3000))
		Console::WriteLine("FAIL: LiDAR thread didn't exit after shutdown");
	else if (quitPressed)
		Console::WriteLine("PASS: LiDAR exited after q");
	else
		Console::WriteLine("LiDAR thread exited on its own (check the messages above)");
	Console::ReadKey(true);
	return 0;
}
