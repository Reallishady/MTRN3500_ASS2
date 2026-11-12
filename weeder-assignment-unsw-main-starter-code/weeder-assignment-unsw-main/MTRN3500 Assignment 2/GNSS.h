#pragma once
#using <System.dll>
#include "SMObjects.h"
#include "NetworkedModule.h"

#define CRC32_POLYNOMIAL 0xEDB88320L

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


	unsigned long CRC32Value(int i)
	{
		int j;
		unsigned long ulCRC;
		ulCRC = i;
		for (j = 8; j > 0; j--)
		{
			if (ulCRC & 1)
				ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
			else
				ulCRC >>= 1;
		}
		
		return ulCRC;
	}
	unsigned long CalculateBlockCRC32(
		unsigned long ulCount, /* Number of bytes in the data block */
		unsigned char* ucBuffer) /* Data block */
	{

		unsigned long ulTemp1;
		unsigned long ulTemp2;
		unsigned long ulCRC = 0;
		while (ulCount-- != 0)
		{

			ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
			ulTemp2 = CRC32Value(((int)ulCRC ^ * ucBuffer++) & 0xff);
			ulCRC = ulTemp1 ^ ulTemp2;
		}
		return(ulCRC);
	}
};