#pragma once
#include <numbers>
#include <frc/geometry/Rotation2d.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <ctre/Phoenix.h>
#include <units/length.h>
#include <units/angle.h>

using angularVelocity = units::compound_unit<units::radians, units::inverse<units::seconds>>;
using angularAcceleration = units::compound_unit<angularVelocity, units::inverse<units::seconds>>;
using kvA_unit = units::compound_unit<units::volts, units::inverse<angularVelocity>>;
using kaA_unit = units::compound_unit<units::volts, units::inverse<angularAcceleration>>;
class SwerveModule
{
    public:
    SwerveModule(int driveChannel, int turnChannel,int encoderChannel, units::volt_t kSAngular, units::unit_t<kvA_unit> kVAngular, bool invert);
    frc::SwerveModuleState getState();
    frc::SwerveModulePosition getPosition();
    double getTurnEncoderCnt();
    double getDegrees();
    void setDesiredState(const frc::SwerveModuleState& state);
    static constexpr auto kMaxAngularVelocity =
      std::numbers::pi * 1_rad / 1_s;  // radians per second
    static constexpr auto kMaxAngularAcceleration =
      std::numbers::pi * 2_rad / 1_s / 1_s;  // radians per second^2
    double getCanCoder();
    CANCoderConfiguration CANCoderConfig;
    void configAngleEncoder();

    void resetEncoder();
    
    private:
    WPI_TalonFX driveMotor;
    WPI_TalonFX turnMotor;
    CANCoder angleEncoder;
    TalonFXConfiguration turnConfig;
    frc2::PIDController drivePID{0.5,0,0};
    frc::ProfiledPIDController<units::radians> turnPID{0.1,0,0, {kMaxAngularVelocity, kMaxAngularAcceleration}};
    frc::SimpleMotorFeedforward<units::meters> driveFeedforward{0.5_V, 3_V/1_mps}; //ks kv
    frc::SimpleMotorFeedforward<units::radians> turnFeedforward; //ksangle kvangle

};