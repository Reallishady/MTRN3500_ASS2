
#include "Controller.h"
#include "VC.h"
#include "CrashAvoidance.h"

#include "GNSS.h"
#include "Lidar.h"
#include "TMM.h"     

using namespace System;
using namespace System::Threading;
using namespace System::Diagnostics;


int main()
{
	ThreadManagement^ TM = gcnew ThreadManagement();
	TM->setupSharedMemory();
	Thread^ TMThread = gcnew Thread(gcnew ThreadStart(TM, &ThreadManagement::threadFunction));
	TMThread->Start();
	TMThread->Join();
	Console::WriteLine("press any key to close");
	Console::ReadKey(true);
	
	return 0;
}
