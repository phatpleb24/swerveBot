#pragma once

#include <numbers>
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <frc/geometry/Pose2d.h>
#include <units/dimensionless.h>
#include <frc/filter/SlewRateLimiter.h>

#include "SwerveModule.h"
#include "Constants.h"

#define sc SwerveConstants

class Drivetrain : public frc2::SubsystemBase
{
    public:
    Drivetrain();
    static constexpr int32_t kUnitsPerRevolution = 2048;
    static constexpr double kGearRatio = 8.45;
    static constexpr units::inch_t kWhellRadiusInches = 3_in;
    static constexpr int k100msPerSecond = 10;
    frc::Field2d m_field;
    bool fieldRelative;

    void Periodic() override;

    void updateOdometry();

    void driveFieldRelative(frc::ChassisSpeeds speeds);

    frc::ChassisSpeeds robotRelativeSpeeds();

    void Drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, units::degrees_per_second_t rot_speed, bool fieldRelative);

    void resetOdometry(frc::Pose2d pose);

    double getPitch();

    double getRoll();

    frc::Pose2d getPose();

    void wheelReset();
    
    frc::SlewRateLimiter<units::dimensionless::scalar> m_xspeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::dimensionless::scalar> m_yspeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::dimensionless::scalar> m_rotLimiter{3 / 1_s};

    private:
    WPI_Pigeon2 gyro{0};
    SwerveModule leftFront{sc::leftFrontDrive, sc::leftFrontTurn, sc::CANCoderLF, sc::angleOffsetLF, true};
    SwerveModule leftBack{sc::leftBackDrive, sc::leftBackTurn, sc::CANCoderLB, sc::angleOffsetLB, true};
    SwerveModule rightFront{sc::rightFrontDrive, sc::rightFrontTurn, sc::CANCoderRF, sc::angleOffsetRF, false};
    SwerveModule rightBack{sc::rightBackDrive,sc::rightBackTurn, sc::CANCoderRB, sc::angleOffsetRB, false};



    //pathplanner::PPHolonomicDriveController swerveDriveController{};
    frc::SwerveDriveOdometry<4> odometry{SwerveConstants::kinematics, gyro.GetRotation2d(), {leftFront.getPosition(), rightFront.getPosition(), leftBack.getPosition(), rightBack.getPosition()}};
};