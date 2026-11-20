#include "CrashAvoidance.h"

using namespace System;
using namespace System::Threading;
using namespace System::Diagnostics;

// all distances in mm. LiDAR frame is x forward, y to the left
const double HALF_WIDTH     = 450.0;  
const double SIDE_WIDTH     = 500.0;   
const double SLOW_DIST      = 1000.0;  
const double STOP_DIST      = 600.0;   
const double MIN_RANGE      = 50.0;    

const double STEER_DEADBAND = 2.0;   
const double STEER_SIGN     = -1.0;    
const double CLEAR          = 1e9;    

CrashAvoidance::CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_LiDAR^ SM_lidar, SM_VehicleControl^ SM_VC)
{
    SM_TM_ = SM_TM;
    SM_LiDAR_ = SM_lidar;
    SM_VC_ = SM_VC;
    watch_ = gcnew Stopwatch;
    laserX_ = gcnew array<double>(STANDARD_LASER_LENGTH);
    laserY_ = gcnew array<double>(STANDARD_LASER_LENGTH);
    speed_ = 0;
    steering_ = 0;
    maxSpeed_ = 0;     
    steerLimited_ = false;
    steerOut_ = 0;
    state_ = -1;    
}

CrashAvoidance::~CrashAvoidance()
{
}

bool CrashAvoidance::getShutdownFlag()
{
    return (SM_TM_->shutdown & bit_CRASHAVOIDANCE) != 0;
}


error_state CrashAvoidance::processHeartbeats()
{
    if ((SM_TM_->heartbeat & bit_CRASHAVOIDANCE) == 0)
    {
        SM_TM_->heartbeat |= bit_CRASHAVOIDANCE;
        watch_->Restart();
    }
    else if (watch_->ElapsedMilliseconds > CRASH_LIMIT)
    {
        Console::WriteLine("CrashAvoidance: TM not responding - shutting down all threads");
        SM_TM_->shutdown = 0xFF;
        return error_state::ERR_TMT_FAILURE;
    }

    return error_state::SUCCESS;
}

void CrashAvoidance::assess()
{

    double steerLeft = STEER_SIGN * steering_;
    int turn = 0;
    if (steerLeft > STEER_DEADBAND)       turn = 1;
    else if (steerLeft < -STEER_DEADBAND) turn = -1;

    double ahead = CLEAR;
    double side = CLEAR;  

    for (int i = 0; i < STANDARD_LASER_LENGTH; i++)
    {
        double x = laserX_[i];
        double y = laserY_[i];
       // double x = 1000;
       // double y = 1000;

        // skip bad readings and anything level with or behind the LiDAR
        if (x <= 0 || Math::Sqrt(x * x + y * y) < MIN_RANGE)
            continue;

        if (Math::Abs(y) <= HALF_WIDTH)
        {
            // inside the UGV's width, we'd hit this going straight
            ahead = Math::Min(ahead, x);
        }
        else if (turn * y > 0 && Math::Abs(y) <= HALF_WIDTH + SIDE_WIDTH)
        {
            // just outside the UGV's width, on the side we're turning towards
            side = Math::Min(side, x);
        }
    }

    // turning into something: straighten the wheels. only when going forward,
    // there's no LiDAR behind so reversing is left to the driver
    steerLimited_ = (side <= SLOW_DIST && speed_ >= 0);
    steerOut_ = steerLimited_ ? 0.0 : steering_;

    // speed cap from whatever's directly ahead: full speed past SLOW_DIST, 0 at STOP_DIST
    double scale = (ahead - STOP_DIST) / (SLOW_DIST - STOP_DIST);
    maxSpeed_ = MAX_SPEED * Math::Max(0.0, Math::Min(1.0, scale));

    // only print when the state changes, otherwise it spams at 50 Hz
    int newState = 0;                              // clear
    if (maxSpeed_ <= 0.0)            newState = 3; // stopped
    else if (steerLimited_)          newState = 2; // steering straightened
    else if (maxSpeed_ < MAX_SPEED)  newState = 1; // slowed down

    if (newState != state_)
    {
        switch (newState)
        {
        case 0: Console::WriteLine("CRASH AVOIDANCE: path clear"); break;
        case 1: Console::WriteLine("CRASH AVOIDANCE: obstacle {0:F0} mm ahead - slowing down", ahead); break;
        case 2: Console::WriteLine("CRASH AVOIDANCE: obstacle {0:F0} mm away on the turning side - steering straightened", side); break;
        case 3: Console::WriteLine("CRASH AVOIDANCE: obstacle {0:F0} mm ahead - stopped", ahead); break;
        }
        state_ = newState;
    }
}

error_state CrashAvoidance::processSharedMemory()
{
    // latest scan
    Monitor::Enter(SM_LiDAR_->lockObject);
    try
    {
        Array::Copy(SM_LiDAR_->x, laserX_, STANDARD_LASER_LENGTH);
        Array::Copy(SM_LiDAR_->y, laserY_, STANDARD_LASER_LENGTH);
    }
    finally
    {
        Monitor::Exit(SM_LiDAR_->lockObject);
    }

    // latest command from the Controller
    Monitor::Enter(SM_VC_->lockObject);
    try
    {
        speed_ = SM_VC_->Speed;
        steering_ = SM_VC_->Steering;
    }
    finally
    {
        Monitor::Exit(SM_VC_->lockObject);
    }

    assess();

    // tell VC
    Monitor::Enter(SM_VC_->lockObject);
    try
    {
        SM_VC_->maxForwardSpeed = maxSpeed_;
        SM_VC_->steerLimited = steerLimited_;
        SM_VC_->steerLimit = steerOut_;
    }
    finally
    {
        Monitor::Exit(SM_VC_->lockObject);
    }

    return error_state::SUCCESS;
}

void CrashAvoidance::threadFunction()
{
    SM_TM_->ThreadBarrier->SignalAndWait();
    watch_->Start();

    while (!getShutdownFlag())
    {
        if (processHeartbeats() != error_state::SUCCESS)
            break;

        processSharedMemory();
        Thread::Sleep(20);
    }

    // fail safe, nothing should drive forward once this thread is gone
    Monitor::Enter(SM_VC_->lockObject);
    try
    {
        SM_VC_->maxForwardSpeed = 0.0;
        SM_VC_->steerLimited = false;
    }
    finally
    {
        Monitor::Exit(SM_VC_->lockObject);
    }

    Console::WriteLine("CrashAvoidance Thread is terminated");
}
