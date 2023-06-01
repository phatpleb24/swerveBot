#pragma once
#include <units/voltage.h>
#include <units/length.h>
#include <units/angle.h>
#include <frc/controller/SimpleMotorFeedForward.h>
#include <ctre/Phoenix.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>

namespace SwerveConstants
{
    const int leftFrontDrive = 1;
    const int leftFrontTurn = 2;
    const int leftBackDrive = 3;
    const int leftBackTurn = 4;
    const int rightFrontDrive = 5;
    const int rightFrontTurn = 6;
    const int rightBackDrive = 7;
    const int rightBackTurn = 8;
    constexpr units::volt_t kS = 0.14643_V;
    constexpr auto kV = 1.8676 * 1_V / 1_mps;
    constexpr auto kA = 0.46219 * 1_V / 1_mps_sq;
    constexpr units::meter_t kTrackWidth = 0.44618_m;
    constexpr units::meter_t kWheelBase = 1_m;
    constexpr auto kVAngular = 2.0807 * 1_V/1_mps;
    constexpr auto kAAngular = 0.39601 * 1_V / 1_mps_sq;
    static frc::SwerveDriveKinematics<4> kinematics{
        frc::Translation2d{kWheelBase/2, kTrackWidth/2}, 
        frc::Translation2d{kWheelBase/2, -kTrackWidth/2}, 
        frc::Translation2d{-kWheelBase/2, kTrackWidth/2}, 
        frc::Translation2d{-kWheelBase/2, -kTrackWidth/2}};
    
    static constexpr units::inch_t kWheelRadiusInches = 3_in;
    static constexpr int32_t kUnitsPerRevolution = 2048;
    static constexpr double kDriveGearRatio = 8.45;
    static constexpr double kAngleGearRatio = 12.8;
    static constexpr int k100msPerSecond = 10; 
};