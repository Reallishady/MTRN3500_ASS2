#pragma once
#using <System.dll>
#include "UGVModule.h"
#include "SMObjects.h"
#include "NetworkedModule.h"

using namespace System;
using namespace System::Threading;
using namespace System::Diagnostics;
using namespace System::IO;
using namespace System::Globalization;

const int  DISPLAY_PORT = 28000;
ref class Display : public NetworkedModule
{
public:
	Display() {};

	Display(SM_ThreadManagement^ SM_TM, SM_LiDAR^ SM_lidar);

	// Send/Recieve data from shared memory structures
	error_state processSharedMemory() override;

	// Shutdown all modules in the software
	void shutdownModules();

	// Get Shutdown signal for module, from Thread Management SM
	bool getShutdownFlag() override;

	// Thread function for TMM
	void threadFunction() override;

	virtual error_state connect(String^ hostName, int portNumber) override;

	virtual error_state communicate() override;

	error_state processHeartbeats();

	

	

private:
	//List of inherited data members
	SM_ThreadManagement^ UGV_SM_TM_;
	SM_LiDAR^ Display_SM_Lidar_;
	Stopwatch^ Display_Stopwatch;

	array<double>^ DisplayX;
	array<double>^ DisplayY;


	void sendDisplayData(array<double> ^ xData, array<double> ^ yData,
		NetworkStream ^ stream)
	{

		// Serialize the data arrays to a byte array 
		
		array<Byte> ^ dataX = gcnew array<Byte>(xData->Length * sizeof(double));
		Buffer::BlockCopy(xData, 0, dataX, 0, dataX->Length);

		array<Byte>^ dataY = gcnew array<Byte>(yData->Length * sizeof(double));
		Buffer::BlockCopy(yData, 0, dataY, 0, dataY->Length);

		// Send byte data over connection 
		Stream->Write(dataX, 0, dataX->Length);
		Thread::Sleep(10);
		Stream->Write(dataY, 0, dataY->Length);
	}
};

