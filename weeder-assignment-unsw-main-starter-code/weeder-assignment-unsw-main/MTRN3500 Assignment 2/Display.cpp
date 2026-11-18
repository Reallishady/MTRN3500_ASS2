#include "Display.h"

Display::Display(SM_ThreadManagement^ SM_TM, SM_LiDAR^ SM_Lidar)
{
	UGV_SM_TM_ = SM_TM;
	Display_SM_Lidar_ = SM_Lidar;
	Display_Stopwatch = gcnew Stopwatch;
	DisplayX = gcnew array<double>(STANDARD_LASER_LENGTH);
	DisplayY = gcnew array<double>(STANDARD_LASER_LENGTH);
}

error_state Display::processHeartbeats()
{
	if ((UGV_SM_TM_->heartbeat & bit_DISPLAY) == 0)
	{
		UGV_SM_TM_->heartbeat |= bit_DISPLAY;
		Display_Stopwatch->Restart();
	}
	else if (Display_Stopwatch->ElapsedMilliseconds > CRASH_LIMIT)
	{
		Console::WriteLine("Display: TM not responding - shutting down all threads");
		shutdownModules();
		return error_state::ERR_TMT_FAILURE;
	}

	return error_state::SUCCESS;
}
// Send/Recieve data from shared memory structures
error_state Display::processSharedMemory()
{
	Monitor::Enter(Display_SM_Lidar_->lockObject);
	try
	{
		
		for (int i = 0; i < STANDARD_LASER_LENGTH; i++)
		{
			DisplayX[i] = -Display_SM_Lidar_->y[i];
			DisplayY[i] = Display_SM_Lidar_->x[i];
		}
	}
	finally
	{
		Monitor::Exit(Display_SM_Lidar_->lockObject);
	}

	return error_state::SUCCESS;
}

void Display::shutdownModules()
{
	UGV_SM_TM_->shutdown = 0xFF;

}

// Get Shutdown signal for module, from Thread Management SM
bool Display::getShutdownFlag()
{
	return (UGV_SM_TM_->shutdown && bit_DISPLAY) != 0;
}

// Thread function for TMM
void Display::threadFunction()
{
	bool connected = (connect(gcnew String(DISPLAY_ADDRESS), DISPLAY_PORT) == error_state::SUCCESS);

	UGV_SM_TM_->ThreadBarrier->SignalAndWait();
	if (!connected)
	{
		Console::WriteLine("Display failed to connect - shutting down all threads");
		shutdownModules();
		return;
	}

	Display_Stopwatch->Start();

	while (!getShutdownFlag())
	{
		if (processHeartbeats() != error_state::SUCCESS)
			break;

		processSharedMemory();

		if (communicate() != error_state::SUCCESS)
		{
			Console::WriteLine("Display lost connection to MATLAB - shutting down all threads");
			shutdownModules();
			break;
		}

		Thread::Sleep(50);
	}

	try
	{
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();
	}
	catch (Exception^)
	{
	}

	Console::WriteLine("Display Thread is terminated");

}



error_state Display::communicate()
{
	try
	{
		sendDisplayData(DisplayX, DisplayY, Stream);
	}
	catch (System::IO::IOException^)
	{
		return error_state::ERR_CONNECTION;
	}

	return error_state::SUCCESS;
}

error_state Display::connect(String^ hostName, int portNumber)
{
	try
	{
		Client = gcnew TcpClient(hostName, portNumber);
		Client->NoDelay = true;
		Client->SendTimeout = 500;
		Client->SendBufferSize = 8192;   
		Stream = Client->GetStream();
	}
	catch (Exception^ e)
	{
		Console::WriteLine("Display connect failed: {0}", e->Message);
		return error_state::ERR_CONNECTION;
	}

	Console::WriteLine("Display connected to MATLAB");
	return error_state::SUCCESS;
}