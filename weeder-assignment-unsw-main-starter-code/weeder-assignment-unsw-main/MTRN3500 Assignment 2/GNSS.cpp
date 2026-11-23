#include "GRU01.h"

using namespace System;
using namespace System::Threading;
using namespace System::Diagnostics;
using namespace System::Text;
using namespace System::IO;

static const int GRU01_PORT = 26000;
static const int GRU01_TIMEOUT_MS = 200;
static const int LINE_LIMIT = 512;     
static const int STALE_MS = 2000;      
static const int GGA_FIELDS = 15;     

GRU01::GRU01(SM_ThreadManagement^ SM_TM, SM_GRU01^ SM_gru)
{
	SM_TM_ = SM_TM;
	SM_GRU01_ = SM_gru;
	GRU01_Stopwatch = gcnew Stopwatch;
	sinceGoodGGA_ = gcnew Stopwatch;
	lineBuffer_ = gcnew StringBuilder;
	fields_ = nullptr;
	hadFix_ = true;   
}

bool GRU01::getShutdownFlag()
{
	return (SM_TM_->shutdown & bit_GRU01) != 0;
}

// non-critical, so if the TM dies this just stops instead of shutting everything down
error_state GRU01::processHeartbeats()
{
	if ((SM_TM_->heartbeat & bit_GRU01) == 0)
	{
		SM_TM_->heartbeat |= bit_GRU01;
		GRU01_Stopwatch->Restart();
	}
	else if (GRU01_Stopwatch->ElapsedMilliseconds > CRASH_LIMIT)
	{
		Console::WriteLine("GR-U01: TM not responding, terminating");
		return error_state::ERR_TMT_FAILURE;
	}

	return error_state::SUCCESS;
}

void GRU01::threadFunction()
{
	bool connected = (connect(gcnew String(WEEDER_ADDRESS), GRU01_PORT) == error_state::SUCCESS);

	// still have to hit the barrier even if connect failed, or everyone else waits forever
	SM_TM_->ThreadBarrier->SignalAndWait();
	if (!connected)
	{
		// heartbeat stops, TM sees it and restarts this thread
		Console::WriteLine("GR-U01 failed to connect, thread stopping");
		disconnect();
		return;
	}

	GRU01_Stopwatch->Start();
	sinceGoodGGA_->Restart();

	while (!getShutdownFlag())
	{
		if (processHeartbeats() != error_state::SUCCESS)
			break;

		error_state comm = communicate();
		if (comm == error_state::ERR_CONNECTION)
		{
			Console::WriteLine("GR-U01 lost connection, thread stopping");
			break;
		}

		// read -> pull out a valid GGA -> convert and store
		if (comm == error_state::SUCCESS && checkData() == error_state::SUCCESS)
			processSharedMemory();

		// still connected but nothing usable for a while, don't keep the old position marked as valid
		if (sinceGoodGGA_->ElapsedMilliseconds > STALE_MS)
			markNoFix("no fresh GGA");

		Thread::Sleep(20);
	}

	markNoFix("thread stopped");
	disconnect();
	Console::WriteLine("GR-U01 Thread is terminated");
}

error_state GRU01::connect(String^ hostName, int portNumber)
{
	Console::WriteLine("GR-U01 connecting to {0}:{1}...", hostName, portNumber);

	try
	{
		Client = gcnew TcpClient(hostName, portNumber);
		Client->NoDelay = true;
		Client->ReceiveTimeout = 500;
		Client->SendTimeout = 500;

		Stream = Client->GetStream();
		ReadData = gcnew array<unsigned char>(1024);

		// new connection, so throw away any half line from the old one
		lineBuffer_->Clear();
	}
	catch (Exception^ e)
	{
		Console::WriteLine("GR-U01 connect failed: {0}", e->Message);
		return error_state::ERR_CONNECTION;
	}

	Console::WriteLine("GR-U01 connected");
	return error_state::SUCCESS;
}

// just grabs whatever text has arrived and adds it to the buffer.
// a read can stop halfway through a sentence so nothing gets parsed here
error_state GRU01::communicate()
{
	try
	{
		// only read when there's data, so this can't block past the timeout
		Stopwatch^ timer = Stopwatch::StartNew();
		while (!Stream->DataAvailable)
		{
			if (getShutdownFlag() || timer->ElapsedMilliseconds > GRU01_TIMEOUT_MS)
				return error_state::ERR_NO_DATA;
			Thread::Sleep(5);
		}

		int n = Stream->Read(ReadData, 0, ReadData->Length);
		if (n == 0)
			return error_state::ERR_CONNECTION;   // 0 bytes = server closed

		lineBuffer_->Append(Encoding::ASCII->GetString(ReadData, 0, n));
	}
	catch (IOException^)
	{
		return error_state::ERR_CONNECTION;
	}
	catch (ObjectDisposedException^)
	{
		return error_state::ERR_CONNECTION;
	}

	return error_state::SUCCESS;
}

// splits the buffer into complete lines (ending in \n) and keeps the newest valid GGA
error_state GRU01::checkData()
{
	String^ text = lineBuffer_->ToString();
	int lastLF = text->LastIndexOf('\n');

	if (lastLF < 0)
	{
		// no full line yet. if it's got this long without one it's junk
		if (text->Length > LINE_LIMIT)
		{
			Console::WriteLine("GR-U01: no line ending in {0} chars, buffer cleared", text->Length);
			lineBuffer_->Clear();
		}
		return error_state::ERR_NO_DATA;
	}

	// everything after the last \n is an unfinished line, keep it for next read
	lineBuffer_->Clear();
	lineBuffer_->Append(text->Substring(lastLF + 1));

	array<String^>^ lines = text->Substring(0, lastLF)->Split('\n');
	error_state result = error_state::ERR_NO_DATA;

	for each (String^ raw in lines)
	{
		String^ line = raw->Trim();   // gets rid of the \r

		// cut off anything before the $
		int dollar = line->IndexOf('$');
		if (dollar < 0)
			continue;
		line = line->Substring(dollar);

		// only want GGA (from any talker, GP/GN/etc), skip RMC, GSV and the rest
		if (line->Length < 7 || line->Substring(3, 3) != "GGA")
			continue;

		array<String^>^ f;
		if (validSentence(line, f))
		{
			fields_ = f;
			result = error_state::SUCCESS;
		}
		else
		{
			Console::WriteLine("GR-U01: rejected {0}", line);
		}
	}

	return result;
}

// checks the sentence looks like $TTSSS,...*HH and the checksum matches
bool GRU01::validSentence(String^ line, array<String^>^% fields)
{
	if (line->Length == 0 || line[0] != '$')
		return false;

	// exactly one * with exactly 2 hex digits after it
	int star = line->IndexOf('*');
	if (star < 0 || line->IndexOf('*', star + 1) >= 0)
		return false;

	String^ supplied = line->Substring(star + 1);
	if (supplied->Length != 2 || !Uri::IsHexDigit(supplied[0]) || !Uri::IsHexDigit(supplied[1]))
		return false;

	// checksum = XOR of every char between $ and *, has to match the hex after the *
	int actual = 0;
	for (int i = 1; i < star; i++)
		actual ^= line[i];

	if (actual != Convert::ToInt32(supplied, 16))
		return false;

	// Split keeps empty fields, so ",," doesn't shift everything after it
	fields = line->Substring(1, star - 1)->Split(',');

	// address is 5 uppercase letters/digits. $P... is manufacturer specific so skip it
	String^ address = fields[0];
	if (address->Length != 5 || address[0] == 'P')
		return false;
	for each (wchar_t c in address)
	{
		if (!Char::IsDigit(c) && !(c >= 'A' && c <= 'Z'))
			return false;
	}

	// has to be GGA with all its fields, and fix quality can't be empty
	if (address->Substring(2) != "GGA" || fields->Length < GGA_FIELDS || fields[6]->Length == 0)
		return false;

	return true;
}

// ddmm.mmmm (lat) or dddmm.mmmm (lon) -> decimal degrees, negative for S/W
// e.g. 3355.0536 S -> -(33 + 55.0536/60) = -33.91756
bool GRU01::toDecimalDegrees(String^ value, String^ direction, bool isLatitude, double% result)
{
	if (value->Length == 0 || direction->Length == 0)
		return false;

	if (isLatitude ? (direction != "N" && direction != "S") : (direction != "E" && direction != "W"))
		return false;

	// lat has 2 degree digits, lon has 3 (goes up to 180)
	int width = isLatitude ? 2 : 3;
	int dot = value->IndexOf('.');
	int wholeDigits = (dot < 0) ? value->Length : dot;
	if (wholeDigits != width + 2)
		return false;

	// only digits and at most one decimal point
	int dots = 0;
	for each (wchar_t c in value)
	{
		if (c == '.') dots++;
		else if (!Char::IsDigit(c)) return false;
	}
	if (dots > 1)
		return false;

	int degrees = Int32::Parse(value->Substring(0, width));
	double minutes = Double::Parse(value->Substring(width));

	// minutes have to be under 60 and degrees within range, otherwise it's corrupted
	int maxDegrees = isLatitude ? 90 : 180;
	if (minutes < 0 || minutes >= 60 || degrees > maxDegrees || (degrees == maxDegrees && minutes != 0))
		return false;

	result = degrees + minutes / 60.0;
	if (direction == "S" || direction == "W")
		result = -result;

	return true;
}

// fix quality number -> words for the printout
String^ GRU01::qualityName(int quality)
{
	switch (quality)
	{
	case 0: return "no fix";
	case 1: return "autonomous";
	case 2: return "differential";
	case 3: return "PPS";
	case 4: return "RTK fixed";
	case 5: return "RTK float";
	case 6: return "estimated";
	case 7: return "manual";
	case 8: return "simulator";
	default: return "unknown";
	}
}

// marks the position as not valid in SM, only prints when the fix is first lost
void GRU01::markNoFix(String^ reason)
{
	Monitor::Enter(SM_GRU01_->lockObject);
	try
	{
		SM_GRU01_->ValidFix = false;
	}
	finally
	{
		Monitor::Exit(SM_GRU01_->lockObject);
	}

	if (hadFix_)
		Console::WriteLine("GR-U01: no valid fix available ({0})", reason);
	hadFix_ = false;
}

// GGA fields: 0 address, 1 UTC, 2-3 lat + N/S, 4-5 lon + E/W, 6 fix quality,
// 7 sats, 8 HDOP, 9-10 altitude + M, 11-12 geoid sep + M, 13-14 DGPS stuff
error_state GRU01::processSharedMemory()
{
	array<String^>^ f = fields_;

	int quality;
	if (!Int32::TryParse(f[6], quality))
		return error_state::ERR_INVALID_DATA;

	// quality 0 = no fix, even if the coordinates are still filled in
	if (quality == 0)
	{
		sinceGoodGGA_->Restart();   // receiver's still talking, it just has no fix
		markNoFix("GGA quality 0");
		return error_state::SUCCESS;
	}

	double lat, lon;
	if (!toDecimalDegrees(f[2], f[3], true, lat) || !toDecimalDegrees(f[4], f[5], false, lon))
	{
		Console::WriteLine("GR-U01: bad coordinates in GGA, ignored");
		return error_state::ERR_INVALID_DATA;
	}

	// altitude above sea level, has to be there and in metres
	double alt;
	if (f[9]->Length == 0 || f[10] != "M" || !Double::TryParse(f[9], alt))
	{
		Console::WriteLine("GR-U01: missing or non-metre altitude in GGA, ignored");
		return error_state::ERR_INVALID_DATA;
	}

	// everything checked out, store it for other modules
	Monitor::Enter(SM_GRU01_->lockObject);
	try
	{
		SM_GRU01_->Latitude = lat;
		SM_GRU01_->Longitude = lon;
		SM_GRU01_->Altitude = alt;
		SM_GRU01_->ValidFix = true;
	}
	finally
	{
		Monitor::Exit(SM_GRU01_->lockObject);
	}

	sinceGoodGGA_->Restart();
	hadFix_ = true;

	// hhmmss.ss -> hh:mm:ss.ss
	String^ utc = f[1];
	if (utc->Length >= 6)
		utc = utc->Substring(0, 2) + ":" + utc->Substring(2, 2) + ":" + utc->Substring(4);

	Console::WriteLine("GR-U01  UTC {0}  Lat: {1,12:F7} deg  Lon: {2,12:F7} deg  Alt (MSL): {3,8:F2} m  [{4}, {5} sats]",
		utc, lat, lon, alt, qualityName(quality), f[7]);

	return error_state::SUCCESS;
}

void GRU01::disconnect()
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
