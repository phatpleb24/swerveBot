
#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>

#define modulePos {leftFront.getPosition(), rightFront.getPosition(), leftBack.getPosition(), rightBack.getPosition()}

frc::SwerveModuleState localFL;
frc::SwerveModuleState localFR;
frc::SwerveModuleState localBL;
frc::SwerveModuleState localBR;

Drivetrain::Drivetrain()
{
    fieldRelative = false;
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
    if(localFL.angle != fl.angle || localFL.speed != fl.speed)
    {
        localFL = fl;
        printf("FL (Speed: %f, Angle: %f)\n", fl.speed.value(), fl.angle.Degrees().value());
    }
    if(localFR.angle != fr.angle || localFR.speed != fr.speed)
    {
        localFR = fr;
        printf("FR (Speed: %f, Angle: %f)\n", fr.speed.value(), fr.angle.Degrees().value());
    }
    if(localBL.angle != bl.angle || localBL.speed != bl.speed)
    {
        localBL = bl;
        printf("BL (Speed: %f, Angle: %f)\n", bl.speed.value(), bl.angle.Degrees().value());
    }
    if(localBR.angle != br.angle || localBR.speed != br.speed)
    {
        localBR = br;
        printf("BR (Speed: %f, Angle: %f)\n", br.speed.value(), br.angle.Degrees().value());
    }
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
    frc::SmartDashboard::PutNumber("Right Back Deg", rightBack.getPosition().angle.Degrees().value());
    frc::SmartDashboard::PutNumber("Right Back Raw", rightBack.getTurnEncoderCnt());
    frc::SmartDashboard::PutNumber("Right Back Deg Falcon", rightBack.getDegrees());
    frc::SmartDashboard::PutBoolean("Field Relative", fieldRelative);
    //printf("Right Back Deg: %f\n", rightBack.getPosition().angle.Degrees().value());
}
