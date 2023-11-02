#pragma once
#include <ctre/Phoenix.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ArmFeedforward.h> 
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Conversions.h"
#include "Constants.h"

class Intake : public frc2::SubsystemBase
{
    public:
    units::degree_t maxRange = 90_deg;
    units::degree_t minRange = 30_deg;
    static constexpr auto kS = 0_V;
    static constexpr auto kG = 0.72_V;
    static constexpr auto kV = 0.48_V * 1_s / 1_rad;
    static constexpr auto kA = 0.02_V * 1_s * 1_s / 1_rad;
    static constexpr double gearRatio = 25.92;
    int intakeState = 0;
    Intake();

    double getEncoderValue();

    void Periodic() override;

    void intakeSpin(double x);

    void wristSpin(double x);

    void moveWrist();

    void setSetpoint(units::degree_t x);

    units::degree_t getPos();

    units::degree_t getPoint();

    private:
    units::degree_t setPoint = 80_deg;
    int horizontalPosFalcon = 0;
    double maxGravityFF = .07;

    WPI_TalonFX intakeMotor{11};
    WPI_TalonFX wristMotor{0};

    //frc::PIDController wristPID{0.9,0,0};
    //frc::ArmFeedforward feedforward{kS, kG, kV, kA};  
};