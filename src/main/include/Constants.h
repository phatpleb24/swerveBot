#pragma once
#include <units/voltage.h>
#include <units/length.h>
#include <units/angle.h>
#include <frc/controller/SimpleMotorFeedForward.h>
#include <ctre/Phoenix.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>
#include <units/angular_velocity.h>

namespace SwerveConstants
{
    const int leftFrontDrive = 4;
    const int leftFrontTurn = 2;
    const int leftBackDrive = 8;
    const int leftBackTurn = 7;
    const int rightFrontDrive = 5;
    const int rightFrontTurn = 9;
    const int rightBackDrive = 3;
    const int rightBackTurn = 1;
    constexpr units::volt_t kS = 0.14643_V;
    constexpr auto kV = 1.8676 * 1_V / 1_mps;
    constexpr auto kA = 0.46219 * 1_V / 1_mps_sq;
    constexpr units::meter_t kTrackWidth = 23.25_in;
    constexpr units::meter_t kWheelBase = 23.25_in;
    constexpr auto kVAngular = 2.0807 * 1_V/1_mps;
    constexpr auto kAAngular = 0.39601 * 1_V / 1_mps_sq;
    static frc::SwerveDriveKinematics<4> kinematics{
        frc::Translation2d{kWheelBase/2, kTrackWidth/2}, 
        frc::Translation2d{kWheelBase/2, -kTrackWidth/2}, 
        frc::Translation2d{-kWheelBase/2, kTrackWidth/2}, 
        frc::Translation2d{-kWheelBase/2, -kTrackWidth/2}};
    constexpr units::volt_t RBkSAngular = 0.012_V;
    constexpr auto RBkVAngular = 0.7_V / 1_rad_per_s;
    static constexpr units::inch_t kWheelRadiusInches = 3_in;
    static constexpr int32_t kUnitsPerRevolution = 2048;
    static constexpr double kDriveGearRatio = 6.55;
    static constexpr double kAngleGearRatio = 10.28;
    static constexpr int k100msPerSecond = 10; 
};