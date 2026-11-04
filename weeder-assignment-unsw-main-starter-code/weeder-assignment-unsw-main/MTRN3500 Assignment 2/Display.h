#pragma once
#include <UGVModule.h>
#include <NetworkedModule.h>

ref class Display : public NetworkedModule
{
public:
	Display() {};

	Display(SM_ThreadManagement^ SM_TM, SM_LiDAR^ SM_lidar, SM_GNSS^ SM_gnss);

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

	void sendDisplayData();

	void FakeLiDAR();

private:
	//List of inherited data members
	//SM_ThreadManagement^ UGV_SM_TM_;
	//TcpClient^ Client;				
	//NetworkStream^ Stream;			
	//array<unsigned char>^ ReadData;

	SM_LiDAR^ Display_SM_LiDAR_;
	SM_GNSS^ Display_SM_GNSS_;
};

