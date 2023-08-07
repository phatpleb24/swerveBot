
#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>

#define modulePos {leftFront.getPosition(), rightFront.getPosition(), leftBack.getPosition(), rightBack.getPosition()}

Drivetrain::Drivetrain()
{
    fieldRelative = true;
    gyro.Reset();
    frc::SmartDashboard::PutData("Field", &m_field);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::degrees_per_second_t rot, bool fieldRelative)
{
    auto states = SwerveConstants::kinematics.ToSwerveModuleStates(fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.GetRotation2d()) : frc::ChassisSpeeds{xSpeed, ySpeed, rot});
    SwerveConstants::kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);
    auto [fl, fr, bl, br] = states;
    leftFront.setDesiredState(fl);
    rightFront.setDesiredState(fr);
    leftBack.setDesiredState(bl);
    rightBack.setDesiredState(br);
}

void Drivetrain::updateOdometry()
{
    odometry.Update(gyro.GetRotation2d(), 
        modulePos);
}

void Drivetrain::resetOdometry(frc::Pose2d pose)
{
    odometry.ResetPosition(gyro.GetRotation2d(), modulePos, pose);
}

frc::Pose2d Drivetrain::getPose()
{
    return odometry.GetPose();
}

void Drivetrain::Periodic()
{
    updateOdometry();
    m_field.SetRobotPose(getPose());
}