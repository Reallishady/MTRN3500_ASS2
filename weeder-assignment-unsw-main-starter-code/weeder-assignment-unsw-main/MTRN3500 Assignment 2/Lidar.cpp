#include "Lidar.h"
static const char* STUDENT_ID = "5492432";
//const char* LIDAR_ADDRESS = "192.168.1.200";
const char* LIDAR_ADDRESS = "127.0.0.1";
static bool countWarned = false;
LiDAR::LiDAR(SM_ThreadManagement^ SM_TM, SM_LiDAR^ SM_lidar)
{
	LiDAR_SM_TM_ = SM_TM;
	LiDAR_SM_LiDAR_ = SM_lidar;
	LiDAR_Stopwatch = gcnew Stopwatch;
	ParsedX = gcnew array<double>(NUM_POINTS);
	ParsedY = gcnew array<double>(NUM_POINTS);
	DistIndex = -1;
}

error_state LiDAR::setupShareMemory()
{
	return error_state::SUCCESS;
}
error_state LiDAR::processHeartbeats()
{
	
	if ((LiDAR_SM_TM_->heartbeat & bit_LiDAR) == 0)
	{

		LiDAR_SM_TM_->heartbeat |= bit_LiDAR;
		LiDAR_Stopwatch->Restart();
	}
	else
	{
		
		if (LiDAR_Stopwatch->ElapsedMilliseconds > CRASH_LIMIT)
		{
			shutdownThreads();
			return error_state::ERR_TMT_FAILURE;
		}
	}
	return error_state::SUCCESS;
}
void LiDAR::shutdownThreads()
{
	LiDAR_SM_TM_->shutdown = 0xFF;
}

bool LiDAR::getShutdownFlag()
{
	return (LiDAR_SM_TM_->shutdown & bit_LiDAR) != 0;
}
void LiDAR::threadFunction() {
	bool connected = (connect(gcnew String(LIDAR_ADDRESS), LIDAR_PORT) == error_state::SUCCESS);

	LiDAR_SM_TM_->ThreadBarrier->SignalAndWait();
	if (!connected)
	{
		Console::WriteLine("LiDAR failed to connect - shutting down all threads");
		shutdownThreads();
		disconnect();
		return;
	}

	LiDAR_Stopwatch->Start();
	Stopwatch^ sinceGoodScan = Stopwatch::StartNew();
	Stopwatch^ printTimer = Stopwatch::StartNew();
	while (!getShutdownFlag())
	{
		if (processHeartbeats() != error_state::SUCCESS)
			break;

		if (communicate() == error_state::SUCCESS &&
			checkData() == error_state::SUCCESS &&
			processSharedMemory() == error_state::SUCCESS)
		{
			sinceGoodScan->Restart();

			if (printTimer->ElapsedMilliseconds > 500)
			{
				printScan();
				printTimer->Restart();
			}

		}
		else if (getShutdownFlag())
		{
			break;      
		}
		else if (sinceGoodScan->ElapsedMilliseconds > MAX_INVALID_SCAN_MS)
		{
			Console::WriteLine("LiDAR lost - shutting down all threads");
			shutdownThreads();
			break;
		}

		Thread::Sleep(20);
	}

	disconnect();
	Console::WriteLine("LiDAR Thread is terminated");
}
error_state LiDAR::connect(String^ hostName, int portNumber)
{
	Console::WriteLine("LiDAR connecting to {0}:{1}...", hostName, portNumber);

	try
	{
		Client = gcnew TcpClient(hostName, portNumber);
		Client->NoDelay = true;
		Client->ReceiveTimeout = 5000;
		Client->SendTimeout = 500;
		Client->ReceiveBufferSize = 8192;
		Client->SendBufferSize = 1024;

		Stream = Client->GetStream();
		ReadData = gcnew array<unsigned char>(8192);

		array<unsigned char>^ id = Encoding::ASCII->GetBytes(gcnew String(STUDENT_ID) + "\n");
		Stream->Write(id, 0, id->Length);

		int n = Stream->Read(ReadData, 0, ReadData->Length);
		String^ reply = Encoding::ASCII->GetString(ReadData, 0, n);
		Console::WriteLine("LiDAR auth reply: [{0}]", reply->Replace("\r", "\\r")->Replace("\n", "\\n"));

		if (!reply->StartsWith("OK"))
		{
			Console::WriteLine("LiDAR auth failed");
			return ERR_CONNECTION;
		}
	}
	catch (Exception^ e)
	{
		Console::WriteLine("LiDAR connect failed: {0}", e->Message);
		return ERR_CONNECTION;
	}

	Console::WriteLine("Connected successfully.");
	return SUCCESS;
}
error_state LiDAR::communicate()
{
	try
	{
		Response = exchange("sRN LMDscandata");
	}
	catch (IOException^)
	{
		return error_state::ERR_CONNECTION;
	}

	if (Response == nullptr)
		return error_state::ERR_NO_DATA;

	return error_state::SUCCESS;
}
void LiDAR::sendCommand(String^ cmd)
{
	array<unsigned char>^ payload = Encoding::ASCII->GetBytes(cmd);
	array<unsigned char>^ frame = gcnew array<unsigned char>(payload->Length + 2);
	frame[0] = 0x02;
	Array::Copy(payload, 0, frame, 1, payload->Length);
	frame[frame->Length - 1] = 0x03;
	Stream->Write(frame, 0, frame->Length);
}

String^ LiDAR::readTelegram()
{
	int total = 0;
	Stopwatch^ timer = Stopwatch::StartNew();

	while (total < ReadData->Length && timer->ElapsedMilliseconds < SCAN_TIMEOUT_MS)
	{
		if (getShutdownFlag())
			return nullptr;

		if (!Stream->DataAvailable)
		{
			Thread::Sleep(5);
			continue;
		}

		int n = Stream->Read(ReadData, total, ReadData->Length - total);
		if (n == 0)
			throw gcnew IOException("Connection closed by server");
		total += n;

		if (ReadData[total - 1] == 0x03)
			break;
	}


	if (total == 0 || ReadData[total - 1] != 0x03)
		return nullptr;


	int start = 0;
	for (int i = 0; i < total; i++)
	{
		if (ReadData[i] == 0x02)
		{
			start = i + 1;
			break;
		}
	}
	return Encoding::ASCII->GetString(ReadData, start, total - 1 - start);
}

String^ LiDAR::exchange(String^ cmd)
{
	flushInput();
	sendCommand(cmd);
	return readTelegram();
}
error_state LiDAR::checkData()
{
	if (Response == nullptr)
		return error_state::ERR_INVALID_DATA;

	Fragments = Response->Split(gcnew array<wchar_t>{ ' ', '\t', '\r', '\n' },
		StringSplitOptions::RemoveEmptyEntries);


	DistIndex = -1;
	for (int i = 0; i < Fragments->Length; i++)
	{
		if (Fragments[i] == "DIST1")
		{
			DistIndex = i;
			break;
		}
	}
	if (DistIndex < 0 || DistIndex + 6 > Fragments->Length)
		return error_state::ERR_INVALID_DATA;

	int count;
	if (!Int32::TryParse(Fragments[DistIndex + 5], NumberStyles::HexNumber,
		CultureInfo::InvariantCulture, count))
		return error_state::ERR_INVALID_DATA;

	if (count != NUM_POINTS)
	{
		if (!countWarned)       
		{
			Console::WriteLine("LiDAR: expected {0} points but telegram has {1}", NUM_POINTS, count);
			countWarned = true;
		}
		return error_state::ERR_INVALID_DATA;
	}

	if (DistIndex + 6 + count > Fragments->Length)
		return error_state::ERR_INVALID_DATA;

	return error_state::SUCCESS;
}


error_state LiDAR::processSharedMemory()
{

	try
	{
		int d = DistIndex;
		double scale = hexToFloat(Fragments[d + 1]);
		double offset = hexToFloat(Fragments[d + 2]);
		int startAngle = Convert::ToInt32(Fragments[d + 3], 16);   
		int step = Convert::ToInt32(Fragments[d + 4], 16);  

		for (int i = 0; i < NUM_POINTS; i++)
		{
			UInt32 raw = Convert::ToUInt32(Fragments[d + 6 + i], 16);
			double range = (raw * scale + offset);
			double angle = (startAngle + i * step) / 10000.0 * Math::PI / 180.0;

			ParsedX[i] = range * Math::Sin(angle);
			ParsedY[i] = -range * Math::Cos(angle);
		}
	}
	catch (Exception^)    
	{
		return error_state::ERR_INVALID_DATA;
	}

	// Lock held only for the copy
	Monitor::Enter(LiDAR_SM_LiDAR_->lockObject);
	try
	{
		Array::Copy(ParsedX, LiDAR_SM_LiDAR_->x, NUM_POINTS);
		Array::Copy(ParsedY, LiDAR_SM_LiDAR_->y, NUM_POINTS);
	}
	finally
	{
		Monitor::Exit(LiDAR_SM_LiDAR_->lockObject);
	}

	return error_state::SUCCESS;
}
