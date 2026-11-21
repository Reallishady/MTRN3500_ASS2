#include "Controller.h"

#define INPUT_TYPE 1 
Controller::Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC)
{
    SM_TM_ = SM_TM;
    SM_VC_ = SM_VC;
    controller_ = new ControllerInterface(1,INPUT_TYPE);
    Controller_Stopwatch = gcnew Stopwatch;
    speed = 0;
    steering = 0;
}

Controller::~Controller()
{
    delete controller_;
}




bool Controller::getShutdownFlag()
{
    return (SM_TM_->shutdown & bit_CONTROLLER) != 0;
}

error_state Controller::processHeartbeats()
{
    if ((SM_TM_->heartbeat & bit_CONTROLLER) == 0)
    {
        SM_TM_->heartbeat |= bit_CONTROLLER;
        Controller_Stopwatch->Restart();
    }
    else if (Controller_Stopwatch->ElapsedMilliseconds > CRASH_LIMIT)
    {
        Console::WriteLine("Controller: TM not responding - shutting down all threads");
        SM_TM_->shutdown = 0xFF;
        return error_state::ERR_TMT_FAILURE;
    }

    return error_state::SUCCESS;
}

error_state Controller::processSharedMemory()
{
    Monitor::Enter(SM_VC_->lockObject);
    try
    {
        SM_VC_->Speed = speed;
        SM_VC_->Steering = steering;
    }
    finally
    {
        Monitor::Exit(SM_VC_->lockObject);
    }

    return error_state::SUCCESS;
}
void Controller::threadFunction()
{
    SM_TM_->ThreadBarrier->SignalAndWait();
    Controller_Stopwatch->Start();

    bool wasConnected = true;

    while (!getShutdownFlag())
    {
        if (processHeartbeats() != error_state::SUCCESS)
            break;

        if (controller_->IsConnected())
        {
            
            controllerState state = controller_->GetState();
            controller_->printControllerState(state);
            if( INPUT_TYPE == 0) {
                speed = (state.rightTrigger - state.leftTrigger) * MAX_SPEED;
            }
            else {
                speed = state.rightThumbY * MAX_SPEED;
            }
          
            
            steering = -(state.rightThumbX * MAX_STEER);

            if (INPUT_TYPE == 0) {
                if (state.buttonA)
                {
                    Console::WriteLine("Controller: A pressed - shutting down all threads");
                    SM_TM_->shutdown = 0xFF;
                }
            }
            

            if (!wasConnected)
            {
                Console::WriteLine("Controller reconnected");
                wasConnected = true;
            }
        }
        else
        {
            speed = 0;
            steering = 0;

            if (wasConnected)
            {
                Console::WriteLine("Controller disconnected - speed and steering set to zero");
                wasConnected = false;
            }
        }

        processSharedMemory();
        Thread::Sleep(20);
    }


    speed = 0;
    steering = 0;
    processSharedMemory();

    Console::WriteLine("Controller Thread is terminated");
}
