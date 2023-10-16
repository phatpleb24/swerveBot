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
    intake->setSetpoint(0_deg);
    startTime = - 1;
    dropFlag = false;  
}
//it looks good but you probably dont have to move the wrist, the wrist should just pop out by itselfs
void PlacementSequence::Execute()
{
    double intakeVolt = 5;
        if(startTime == -1)
        {
            startTime = frc::Timer::GetFPGATimestamp().value();
        }
        /*if(frc::Timer::GetFPGATimestamp().value() - startTime >= 2.5)
        {
            intakeVolt = -9;
        }*/
        //else 
          //  intakeVolt = 0;  
    printf("Execute Placement\n");
    printf("Intake %.03f", intakeVolt);
    //intake->moveWrist();
    intake->intakeSpin(intakeVolt);
}

bool PlacementSequence::IsFinished()
{
    //return intake->getEncoderValue() >= position + 0.1;
    return frc::Timer::GetFPGATimestamp().value() - startTime >= 2;
}

void PlacementSequence::End(bool interrupted)
{
    printf("Placement End\n");
    intake->intakeSpin(0);
}