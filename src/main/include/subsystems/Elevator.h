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

    void setState(units::meter_t goalPoint);

    void SimulationPeriodic() override;

    void Periodic() override;

    void reset();
    /* My code
    //void updateElevatorMeters();

    //void updateSmartDashboard();
    */

    static constexpr units::meter_t kRaisedPosition = -50_in;
    static constexpr units::meter_t kMidPosition = 1_ft;
    static constexpr units::meter_t kLoweredPosition = 0_ft;

    static constexpr units::meter_t kDrumRadius = 0.75_in;
    static constexpr units::kilogram_t kCarriageMass = 4.5_kg;
    static constexpr double kGearRatio = 6.0;
    //static units::meter_t elevatorSetpointMeters;

    static constexpr units::second_t kDt = 20_ms;
    private:
    WPI_TalonFX leftMotor{6};
    WPI_TalonFX rightMotor{10};
    /*new code
    //frc::TrapezoidProfile<units::meters>::Constraints constraints{1.5_fps, 1.5_fps_sq};
    //frc::ProfiledPIDController<units::meters> elevatorPID{1.3, 0.0, 0.7,
    //                                                     constraints, kDt};
        //frc::TrapezoidProfile<units::meters>::State lastProfiledReference;*/
    //old code
    frc::LinearSystem<2,1,1> elevatorPlant = frc::LinearSystemId::ElevatorSystem(frc::DCMotor::Falcon500(2), kCarriageMass, kDrumRadius, kGearRatio);
    frc::KalmanFilter<2,1,1> observer{elevatorPlant, {0.0508, 0.5}, {0.001}, 20_ms};
    frc::LinearQuadraticRegulator<2, 1> controller{elevatorPlant, {0.0254, 0.254}, {12}, 20_ms};
    frc::LinearSystemLoop<2,1,1> loop{elevatorPlant, controller, observer, 12_V, 20_ms};
    frc::TrapezoidProfile<units::meters>::Constraints constraints{1.5_fps, 1.5_fps_sq};
    frc::TrapezoidProfile<units::meters>::State lastProfiledReference;

    TalonFXSimCollection motorSim = leftMotor.GetSimCollection();
    frc::sim::ElevatorSim elevatorSim{frc::DCMotor::Falcon500(2), kGearRatio, kCarriageMass, kDrumRadius, kLoweredPosition, kRaisedPosition, true, {0.01}};
    frc::Mechanism2d mech2d{20,50};
    frc::MechanismRoot2d* elevatorRoot = mech2d.GetRoot("Elevator Root", 10, 0);
    frc::MechanismLigament2d* elevatorMech2d = elevatorRoot->Append<frc::MechanismLigament2d>("Elevator", units::inch_t{elevatorSim.GetPosition()}.value(), 90_deg);
};