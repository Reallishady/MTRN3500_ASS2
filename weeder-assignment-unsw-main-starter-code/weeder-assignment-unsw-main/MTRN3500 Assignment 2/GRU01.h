#pragma once
#using <System.dll>
#include "SMObjects.h"
#include "NetworkedModule.h"

ref class GRU01 : public NetworkedModule
{
public:
	GRU01(SM_ThreadManagement^ SM_TM, SM_GRU01^ SM_gru);

	// from UGVModule
	virtual void threadFunction() override;
	virtual error_state processSharedMemory() override;
	virtual bool getShutdownFlag() override;

	// from NetworkedModule
	virtual error_state connect(System::String^ hostName, int portNumber) override;
	virtual error_state communicate() override;

	error_state checkData();
	error_state processHeartbeats();

private:
	// sentence level: framing, checksum, address. fills fields on success
	bool validSentence(System::String^ line, array<System::String^>^% fields);

	// ddmm.mmmm / dddmm.mmmm + hemisphere -> signed decimal degrees, false if malformed or out of range
	bool toDecimalDegrees(System::String^ value, System::String^ direction, bool isLatitude, double% result);

	System::String^ qualityName(int quality);
	void markNoFix(System::String^ reason);
	void disconnect();

	SM_GRU01^ SM_GRU01_;
	System::Diagnostics::Stopwatch^ GRU01_Stopwatch;   // heartbeat
	System::Diagnostics::Stopwatch^ sinceGoodGGA_;     // freshness of the last valid fix

	System::Text::StringBuilder^ lineBuffer_;   // received text that hasn't reached a LF yet
	array<System::String^>^ fields_;            // fields of the latest GGA that passed validation
	bool hadFix_;                               // so "no valid fix" only prints when it changes
};
