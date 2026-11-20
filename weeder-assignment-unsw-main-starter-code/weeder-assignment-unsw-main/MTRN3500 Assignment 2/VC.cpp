#include "VC.h"

VehicleControl::VehicleControl(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC)
{
	SM_TM_ = SM_TM;
	SM_VC_ = SM_VC;
	VC_Stopwatch = gcnew Stopwatch;
	speed_ = 0;
	steering_ = 0;
	wdog_ = 0;
}

VehicleControl::~VehicleControl()
{
	disconnect();
}

bool VehicleControl::getShutdownFlag()
{
	return (SM_TM_->shutdown & bit_VC) != 0;
}

// critical thread, so if the TM dies everything goes down
error_state VehicleControl::processHeartbeats()
{
	if ((SM_TM_->heartbeat & bit_VC) == 0)
	{
		SM_TM_->heartbeat |= bit_VC;
		VC_Stopwatch->Restart();
	}
	else if (VC_Stopwatch->ElapsedMilliseconds > CRASH_LIMIT)
	{
		Console::WriteLine("VC: TM not responding - shutting down all threads");
		shutdownModules();
		return error_state::ERR_TMT_FAILURE;
	}

	return error_state::SUCCESS;
}

// grab the latest speed/steering the Controller wrote
error_state VehicleControl::processSharedMemory()
{
	Monitor::Enter(SM_VC_->lockObject);
	try
	{
		speed_ = SM_VC_->Speed;
		steering_ = SM_VC_->Steering;

	
		if (speed_ > SM_VC_->maxForwardSpeed)
			speed_ = SM_VC_->maxForwardSpeed;
		if (SM_VC_->steerLimited)
			steering_ = SM_VC_->steerLimit;
	}
	finally
	{
		Monitor::Exit(SM_VC_->lockObject);
	}


	speed_ = Math::Max(-MAX_SPEED_MS, Math::Min(MAX_SPEED_MS, speed_));
	steering_ = Math::Max(-MAX_STEER_DEG, Math::Min(MAX_STEER_DEG, steering_));

	return error_state::SUCCESS;
}

void VehicleControl::threadFunction()
{
	bool connected = (connect(gcnew String(WEEDER_ADDRESS), VC_PORT) == error_state::SUCCESS);

	SM_TM_->ThreadBarrier->SignalAndWait();
	if (!connected)
	{
		Console::WriteLine("VC failed to connect - shutting down all threads");
		shutdownModules();
		disconnect();
		return;
	}

	VC_Stopwatch->Start();

	while (!getShutdownFlag())
	{
		if (processHeartbeats() != error_state::SUCCESS)
			break;

		processSharedMemory();

		if (communicate() != error_state::SUCCESS)
		{
			Console::WriteLine("VC lost connection to UGV - shutting down all threads");
			shutdownModules();
			break;
		}

		Thread::Sleep(50);
	}

	
	speed_ = 0;
	steering_ = 0;
	communicate();

	disconnect();
	Console::WriteLine("VC Thread is terminated");
}

error_state VehicleControl::connect(String^ hostName, int portNumber)
{
	Console::WriteLine("VC connecting to {0}:{1}...", hostName, portNumber);

	try
	{
		Client = gcnew TcpClient(hostName, portNumber);
		Client->NoDelay = true;
		Client->ReceiveTimeout = 500;
		Client->SendTimeout = 500;

		Stream = Client->GetStream();
		ReadData = gcnew array<unsigned char>(64);

	
		array<unsigned char>^ id = Encoding::ASCII->GetBytes(gcnew String(STUDENT_ID) + "\n");
		Stream->Write(id, 0, id->Length);

		int n = Stream->Read(ReadData, 0, ReadData->Length);
		String^ reply = Encoding::ASCII->GetString(ReadData, 0, n);

		if (!reply->StartsWith("OK"))
		{
			Console::WriteLine("VC auth failed");
			return error_state::ERR_CONNECTION;
		}
	}
	catch (Exception^ e)
	{
		Console::WriteLine("VC connect failed: {0}", e->Message);
		return error_state::ERR_CONNECTION;
	}

	Console::WriteLine("VC connected");
	return error_state::SUCCESS;
}


error_state VehicleControl::communicate()
{
	String^ cmd = "# " + steering_ + " " + speed_ + " " + wdog_ + " #";

	try
	{
		array<unsigned char>^ data = Encoding::ASCII->GetBytes(cmd);
		Stream->Write(data, 0, data->Length);
	}
	catch (Exception^)
	{
		return error_state::ERR_CONNECTION;
	}

	wdog_ = 1 - wdog_;   
	return error_state::SUCCESS;
}

