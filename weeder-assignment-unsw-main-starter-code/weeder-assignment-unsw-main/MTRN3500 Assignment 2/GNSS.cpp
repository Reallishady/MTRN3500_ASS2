
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
	bool connected = (connect(gcnew String(WEEDER_ADDRESS), GNSS_PORT) == error_state::SUCCESS);

	
	if (!connected)
	{
		Console::WriteLine("Display failed to connect - shutting down all threads");
		
		return;
	}

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
		
		if (processHeartbeats() != error_state::SUCCESS)
			break;
		if (communicate() == SUCCESS && checkData() == SUCCESS && processSharedMemory() == error_state::SUCCESS)
		{
			processSharedMemory();
		}
		
		
	}

	Console::WriteLine("GNSS Thread is terminated");
}

error_state GNSS::processHeartbeats()
{
	//check if the GNSS bit in the hearbeat byte is low
	if ((GNSS_SM_TM_->heartbeat & bit_GNSS) == 0)
	{
		
		GNSS_SM_TM_->heartbeat |= bit_GNSS;
		GNSS_Stopwatch->Restart();
	}

	else if (GNSS_Stopwatch->ElapsedMilliseconds > CRASH_LIMIT) {

		Console::WriteLine("GNSS: TM not responding");
		return error_state::ERR_TMT_FAILURE;
		
	}
	

	return error_state::SUCCESS;
}


bool GNSS::getShutdownFlag()
{
	return GNSS_SM_TM_->shutdown & bit_GNSS;
}

error_state GNSS::communicate()
{
	try {
		Stopwatch^ GNSS_timer = Stopwatch::StartNew();
		unsigned int header = 0;
		while (header != 0xAA44121C) {
			if (getShutdownFlag() || GNSS_timer->ElapsedMilliseconds > 200) {
				return error_state::ERR_NO_DATA;
			}

			

			int b = Stream->ReadByte();
			if (b < 0) {
				return error_state::ERR_CONNECTION;
			}

			header = (header << 8) | b;
		}

		
		ReadData[0] = 0xAA; ReadData[1] = 0x44; ReadData[2] = 0x12; ReadData[3] = 0x1C;
		int packet_count = 4;

		while (packet_count < GNSS_PACKET_SIZE) {
			int data = Stream->Read(ReadData, packet_count, GNSS_PACKET_SIZE - packet_count);

			if (data == 0) {
				return error_state::ERR_CONNECTION;
			}

			packet_count += data;
		}
	}
	catch (IOException^) {
		return error_state::ERR_NO_DATA;
	}

	return error_state::SUCCESS;
}

error_state GNSS::checkData()
{
	unsigned char buffer[GNSS_PACKET_SIZE];
	for (int i = 0; i < GNSS_PACKET_SIZE;i++) {
		buffer[i] = ReadData[i];
	}
	CalculatedCRC = CalculateBlockCRC32(GNSS_PACKET_SIZE - 4, buffer);
	ReceivedCRC = BitConverter::ToUInt32(ReadData, GNSS_PACKET_SIZE - 4);

	if (CalculatedCRC != ReceivedCRC) {
		Console::WriteLine("CRC check failed");
		return error_state::ERR_INVALID_DATA;
	}

	return error_state::SUCCESS;
}

error_state GNSS::processSharedMemory()
{
	double northing = BitConverter::ToDouble(ReadData, 44);
	double easting = BitConverter::ToDouble(ReadData, 52);
	double height = BitConverter::ToDouble(ReadData, 60);
	Monitor::Enter(GNSS_SM_GNSS_->lockObject);
	try {
		GNSS_SM_GNSS_->Northing = northing;
		GNSS_SM_GNSS_->Easting = easting;
		GNSS_SM_GNSS_->Height = height;
	}
	finally {
		Monitor::Exit(GNSS_SM_GNSS_->lockObject);
	}
	printData(northing, easting, height);
	return error_state::SUCCESS;
}

error_state GNSS::connect(String^ hostName, int portNumber) {
	Console::WriteLine("GNSS connecting to {0}:{1}...", hostName, portNumber);

	try
	{
		Client = gcnew TcpClient(hostName, portNumber);
		Client->NoDelay = true;
		Client->ReceiveTimeout = 500;
		Client->SendTimeout = 500;
		Client->ReceiveBufferSize = 8192;
		Client->SendBufferSize = 1024;

		Stream = Client->GetStream();
		ReadData = gcnew array<unsigned char>(GNSS_PACKET_SIZE);
	
	}
	catch (Exception^ e)
	{
		Console::WriteLine("GNSS connect failed: {0}", e->Message);
		return ERR_CONNECTION;
	}

	Console::WriteLine("Connected successfully.");
	return SUCCESS;
}
	
