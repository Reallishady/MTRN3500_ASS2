#include "Display.h"

Display::Display(SM_ThreadManagement^ SM_TM, SM_LiDAR^ SM_lidar)
{
	UGV_SM_TM_ = SM_TM;
	Display_SM_LiDAR_ = SM_lidar;
	
}

// Send/Recieve data from shared memory structures
error_state Display::processSharedMemory()
{
	return error_state::SUCCESS;
}

// Shutdown all modules in the software
void Display::shutdownModules()
{


}

// Get Shutdown signal for module, from Thread Management SM
bool Display::getShutdownFlag()
{
	return bit_DISPLAY;
}

// Thread function for TMM
void Display::threadFunction()
{

}

error_state Display::connect(String^ hostName, int portNumber)
{
	Client = gcnew TcpClient(hostName, portNumber);
	Stream = Client->GetStream();
	Client->NoDelay = true;
	Client->ReceiveTimeout = 500;
	Client->SendTimeout = 500;
	Client->ReceiveBufferSize = 1024;
	Client->SendBufferSize = 1024;

	SendData = gcnew array<unsigned char>(64);
	ReadData = gcnew array<unsigned char>(64);

	return error_state::SUCCESS;
}

error_state Display::communicate()
{
	sendDisplayData();
	return error_state::SUCCESS;
}



//void Display::sendDisplayData()
//{
//	// Serialize the data arrays to a byte array
//	//(format required for sending)
//
//	Monitor::Enter(Display_SM_LiDAR_->lockObject);
//
//	array<Byte>^ dataX = gcnew array<Byte>(Display_SM_LiDAR_->x->Length * sizeof(double));
//	Buffer::BlockCopy(Display_SM_LiDAR_->x, 0, dataX, 0, dataX->Length);
//	array<Byte>^ dataY = gcnew array<Byte>(Display_SM_LiDAR_->y->Length * sizeof(double));
//	Buffer::BlockCopy(Display_SM_LiDAR_->y, 0, dataY, 0, dataY->Length);
//
//	Monitor::Exit(Display_SM_LiDAR_->lockObject);
//
//	// Send byte data over connection 
//	Stream->Write(dataX, 0, dataX->Length);
//	Thread::Sleep(10);
//	Stream->Write(dataY, 0, dataY->Length);
//}