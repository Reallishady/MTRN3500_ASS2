#pragma once
#using <System.dll>
#include "SMObjects.h"
#include "NetworkedModule.h"


using namespace System;
using namespace System::Threading;
using namespace System::Diagnostics;
using namespace System::IO;
using namespace System::Globalization;



const int    NUM_POINTS = 361;

const int   LIDAR_PORT = 23000;
const int    MAX_INVALID_SCAN_MS = 1000;
const int SCAN_TIMEOUT_MS = 400;

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

	void sendCommand(String^ cmd);
	String^ readTelegram();
	String^ exchange(String^ cmd);

private:
	SM_ThreadManagement^ LiDAR_SM_TM_;
	SM_LiDAR^ LiDAR_SM_LiDAR_;
	Stopwatch^ LiDAR_Stopwatch;
	String^ Response;
	array<String^>^ Fragments;
	int DistIndex;                        
	array<double>^ ParsedX;               
	array<double>^ ParsedY;
	float hexToFloat(String^ hex)
	{
		return BitConverter::ToSingle(BitConverter::GetBytes(Convert::ToUInt32(hex, 16)), 0);
	}
	void LiDAR::disconnect()
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
	void LiDAR::flushInput()
	{
		while (Stream->DataAvailable)
			Stream->Read(ReadData, 0, ReadData->Length);
	}
	void LiDAR::printScan()
	{
		StringBuilder^ sb = gcnew StringBuilder();
		sb->AppendLine("LiDAR scan (x, y)");

		for (int i = 0; i < NUM_POINTS; i++)
		{
			sb->AppendFormat("{0,3}: ({1,6:F0}, {2,6:F0}) mm    ", i, ParsedX[i], ParsedY[i]);
			if ((i + 1) % 4 == 0)
				sb->AppendLine();
		}
		sb->AppendLine();

		Console::Write(sb->ToString());
	}
};