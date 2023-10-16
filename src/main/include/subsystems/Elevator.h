#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystem.h>
#include <ctre/Phoenix.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/velocity.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>

class Elevator : public frc2::SubsystemBase
{
    public:
    Elevator();

    void setState();

    void setPos(units::meter_t goalPoint);

    void Periodic() override;

    void voltage();

    /* My code
    //void updateElevatorMeters();

    //void updateSmartDashboard();
    */

    static constexpr units::meter_t kRaisedPosition = 54_in;
    static constexpr units::meter_t kLoweredPosition = 0_ft;

    static constexpr units::meter_t kDrumRadius = 2.049_in;
    static constexpr units::kilogram_t kCarriageMass = 35_kg;
    static constexpr double kGearRatio = 5.0;
    //static units::meter_t elevatorSetpointMeters;

    static constexpr units::second_t kDt = 20_ms;
    private:
    units::meter_t setpoint = 0_m;
    double gravityFF = .6;
    WPI_TalonFX leftMotor{6};
    WPI_TalonFX rightMotor{10};
};