#include "subsystems/Drivetrain.h"
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "subsystems/Intake.h"

class Balance : public frc2::CommandHelper<frc2::CommandBase, Balance>
{
    public:
    Balance(Drivetrain* swerve);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

    private:
    Drivetrain* swerve;
    bool timerStarted;
    double pitch;
    double timer;
    double maxPitch;
    double maxSpeed;
    double pitchTolerance;
    double balanceDuration;
    double debugTimestamp;
    double levelAngle;
};