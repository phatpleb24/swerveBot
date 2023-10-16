#pragma once

#include "subsystems/Intake.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class IntakeDown : public frc2::CommandHelper<frc2::CommandBase, IntakeDown>
{
    public:
    IntakeDown(Intake* Intake);

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    private:
    Intake* intake;
};