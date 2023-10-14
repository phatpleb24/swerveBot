#pragma once

#include "subsystems/Intake.h"
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class PlacementSequence : public frc2::CommandHelper<frc2::CommandBase, PlacementSequence>
{
    public:
    PlacementSequence(Intake* Intake);

    void Initialize() override;

    void Execute() override;

    void End(bool finished) override;

    bool IsFinished() override;

    private:
    Intake* intake;
    double position;
    double startTime;
    bool dropFlag;
};