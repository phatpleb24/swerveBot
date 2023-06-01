#pragma once
#include <numbers>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <ctre/Phoenix.h>
#include <units/length.h>

class SwerveModule
{
    public:
    SwerveModule(int driveChannel, int turnChannel);
    frc::SwerveModuleState getState();
    frc::SwerveModulePosition getPosition();
    void setDesiredState(const frc::SwerveModuleState& state);
    static constexpr auto kMaxAngularVelocity =
      std::numbers::pi * 1_rad_per_s;  // radians per second
    static constexpr auto kMaxAngularAcceleration =
      std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

    void resetEncoder();
    
    private:
    WPI_TalonFX driveMotor;
    WPI_TalonFX turnMotor;
    frc2::PIDController drivePID{1,0,0};
    frc::ProfiledPIDController<units::radians> turnPID{1,0,0, {kMaxAngularVelocity, kMaxAngularAcceleration}};
    frc::SimpleMotorFeedforward<units::meters> driveFeedforward{1_V, 3_V/1_mps};
    frc::SimpleMotorFeedforward<units::radians> turnFeedforward{1_V, 0.5_V / 1_rad_per_s};

    int DistanceToNativeUnits(units::meter_t position);
    int VelocityToNativeUnits(units::meters_per_second_t velocity);
    units::meters_per_second_t NativeUnitstoVelocityMPS(double sensorCounts);
    units::meter_t NativeUnitsToDistanceMeters(double sensorCounts);
};