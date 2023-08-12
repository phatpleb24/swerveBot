#pragma once
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ArmFeedforward.h> 
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include "Conversions.h"
#include "Constants.h"

class Intake : public frc2::SubsystemBase
{
    public:
    units::degree_t maxRange = 120_deg;
    units::degree_t minRange = 0_deg;
    static constexpr auto kS = 1_V;
    static constexpr auto kG = 1_V;
    static constexpr auto kV = 1_V * 1_s / 1_rad;
    static constexpr auto kA = 1_V * 1_s * 1_s / 1_rad;
    static constexpr double gearRatio = 5;
    Intake();

    void Periodic() override;

    void intakeSpin(double x);

    void wristSpin(double x);

    void wristSetPoint(units::degree_t x);

    units::degree_t getPoint();

    private:
    units::degree_t setPoint = 0_deg;

    WPI_TalonFX intakeMotor{0};
    WPI_TalonFX wristMotor{11};

    frc::PIDController wristPID{0.9,0,0};
    frc::ArmFeedforward feedforward{kS, kG, kV, kA};  
};