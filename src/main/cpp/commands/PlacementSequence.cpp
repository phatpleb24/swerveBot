#include "commands/PlacementSequence.h"
#include <frc/Timer.h>

PlacementSequence::PlacementSequence(Intake* intake) : intake{intake}
{
    printf("Placement Constructor\n");
    AddRequirements({intake});
}

void PlacementSequence::Initialize()
{
    printf("Placement Init\n");
    intake->setSetpoint(90_deg);
    startTime = - 1;
    dropFlag = false;  
}

void PlacementSequence::Execute()
{
    double intakeVolt = 0;
        if(startTime == -1)
        {
            startTime = frc::Timer::GetFPGATimestamp().value();
        }
        if(frc::Timer::GetFPGATimestamp().value() - startTime >= 2.5)
        {
            intakeVolt = -9;
        }
        else 
            intakeVolt = 0;  
    printf("Execute Placement\n");
    printf("Intake %.03f", intakeVolt);
    intake->moveWrist(0_deg);
    intake->intakeSpin(intakeVolt);
}

bool PlacementSequence::IsFinished()
{
    return intake->getEncoderValue() >= position + 0.1;
}

void PlacementSequence::End(bool interrupted)
{
    printf("Placement End\n");
    intake->intakeSpin(0);
}