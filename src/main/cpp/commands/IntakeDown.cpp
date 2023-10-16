#include "commands/IntakeDown.h"

IntakeDown::IntakeDown(Intake* intake) : intake{intake}
{
    AddRequirements({intake});
}

void IntakeDown::Execute()
{
    intake->setSetpoint(0_deg);
}

bool IntakeDown::IsFinished()
{
    return intake->getPos() <= 0_deg;
}

void IntakeDown::End(bool finished)
{
    intake->intakeSpin(-5);
}