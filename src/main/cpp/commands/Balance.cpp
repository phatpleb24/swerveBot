#include "commands/Balance.h"
#include <frc/Timer.h>
Balance::Balance(Drivetrain* swerve) : swerve{swerve}
{
    AddRequirements({swerve});
    printf("Balance Constructor\n");
}

void Balance::Initialize()
{
    printf("Balance Init\n");
    maxPitch = 15;
    maxSpeed = 1.7;
    pitchTolerance = 0.5;
    balanceDuration = 2;
    timerStarted = false;
    debugTimestamp = frc::Timer::GetFPGATimestamp().value();
    levelAngle = 0;
}

void Balance::Execute()
{
    units::meters_per_second_t xSpeed = (units::meters_per_second_t)2;
    units::meters_per_second_t ySpeed = (units::meters_per_second_t)0;
    units::degrees_per_second_t rotSpeed = (units::degrees_per_second_t)0;
    if(swerve->getPitch() > 3)
    {
        swerve->Drive(xSpeed, ySpeed, rotSpeed, true);
    }
    else if(swerve->getPitch() < -3)
    {
        swerve->Drive(-xSpeed, ySpeed, rotSpeed, true);
    }
    else swerve->Drive(ySpeed, ySpeed, rotSpeed, true);
}

bool Balance::IsFinished()
{
    double localTimestamp = frc::Timer::GetFPGATimestamp().value();
    bool printMsg = true;
    if(localTimestamp - debugTimestamp > 1)
    {
        debugTimestamp = localTimestamp;
        printMsg = true;
    }
    if(-swerve->getRoll() > levelAngle-pitchTolerance && -swerve->getRoll() < levelAngle + pitchTolerance)
    {
        if (!timerStarted)
        {
            if(printMsg)
                printf("IsFinished, Start Timer %0.3f\n", localTimestamp);
            timerStarted = true;
            timer = frc::Timer::GetFPGATimestamp().value();
            return false;
        }
        else if(frc::Timer::GetFPGATimestamp().value() - timer > balanceDuration)
        {
            if(printMsg)
                printf("IsFinished, Balance Achieved %0.3f\n", localTimestamp);
            return true;
        }
    }
    else if (timerStarted)
    {
        if(printMsg)
            printf("IsFinished, Timer Canceled %0.3f\n", localTimestamp);
        timerStarted = false;
    }
    return false;
}