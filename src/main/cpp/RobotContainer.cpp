// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/RamseteController.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc/Filesystem.h>
#include <frc/geometry/Pose2d.h>
#include <wpi/fs.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include "commands/PlacementSequence.h"
#include "commands/Balance.h"
#include <units/angle.h>
#include "commands/PlacementSequence.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();

  swerve.SetDefaultCommand(frc2::RunCommand(
    [this]
    {
      if(controller.GetLeftStickButtonPressed()) swerve.fieldRelative = !swerve.fieldRelative;
      //old deadband values all 0.05
      auto xSpeed = swerve.m_xspeedLimiter.Calculate(std::clamp(frc::ApplyDeadband(-controller.GetLeftY(), 0.10),-0.4, 0.4) /** swerve.kMaxSpeed.value()*/);
      auto ySpeed = swerve.m_yspeedLimiter.Calculate(std::clamp(frc::ApplyDeadband(-controller.GetLeftX(), 0.10), -0.4,0.4) /** swerve.kMaxSpeed.value()*/);
      //old rot speed values -0.2 - 0.2
      auto rotSpeed = swerve.m_rotLimiter.Calculate(std::clamp(frc::ApplyDeadband(controller.GetRightX(), 0.10),-0.8,  0.8) /** swerve.kMaxAngularSpeed.value()*/);
      swerve.Drive(xSpeed * swerve.kMaxSpeed, ySpeed * swerve.kMaxSpeed, rotSpeed * swerve.kMaxAngularSpeed, swerve.fieldRelative);
    }, 
    {&swerve}));
  
  elevator.SetDefaultCommand(frc2::RunCommand(
    [this]
    {
      if(controller.GetAButton())
      {
        //elevator.setSetpoint(50_in);
      }
      else if (controller.GetBButton())
      {
        //elevator.setSetpoint(0_in);
      }
      //elevator.raiseElevator();
    }, {&elevator}
  ));

  intake.SetDefaultCommand(frc2::RunCommand(
    [this]
    {
      if(controller2.GetYButton())
      {
        intake.setSetpoint(90_deg);
      }
      else if(controller2.GetXButton()){
        intake.setSetpoint(0_deg);
      }
      intake.moveWrist();
      if(controller2.GetLeftBumper())
      {
        if (intake.intakeState != 1){
          intake.intakeState = 1;
        }
        else {
          intake.intakeState = 0;
        }
      }
      else if(controller2.GetRightBumper())
      {
        if (intake.intakeState != -1){
          intake.intakeState = -1;
        }
        else {
          intake.intakeState = 0;
        }
      } 
      if(intake.intakeState==1){
        intake.intakeSpin(4);
      }
      else if(intake.intakeState==-1){
        intake.intakeSpin(-4);
      }
      else if(intake.intakeState==0){
        intake.intakeSpin(0);
      }
    }, {&intake}
  ));
  Balance* balanceCMD = new Balance(&swerve);
  controller.B().WhileTrue(balanceCMD);
}

void RobotContainer::ConfigureBindings() 
{
 
}

// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//   frc::Trajectory pathWeaverTraj;
//   fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
//   deployDirectory = deployDirectory / "output" / file;
//   pathWeaverTraj = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

//   //m_drive.m_field.GetObject("traj")->SetTrajectory(trajectory);
  
//   drive.resetOdometry(pathWeaverTraj.InitialPose());
//   frc2::RamseteCommand ramseteCommand
//   {
//     pathWeaverTraj,
//     [this]() {return m_drive.getPose();},
//     frc::RamseteController{},
//     frc::SimpleMotorFeedforward<units::meters>{DriveConstants::kS, DriveConstants::kV, DriveConstants::kA},
//     frc::DifferentialDriveKinematics(DriveConstants::kTrackWidth),
//     [this]() {return m_drive.getWheelSpeed();},
//     frc2::PIDController{.5, 0, 0},
//     frc2::PIDController{.5, 0, 0},
//     [this](auto left, auto right){m_drive.tankDriveVolts(left, right);},
//     {&m_drive},
//   };
//   PlacementSequence placeCMD = PlacementSequence(&m_arm);
//   return std::move(placeCMD).ToPtr();//.AndThen(std::move(ramseteCommand).ToPtr()).AndThen([this] {m_drive.tankDriveVolts(0_V,0_V);}, {&m_drive});
// }
frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  PlacementSequence placeCMD = PlacementSequence(&intake);
  return std::move(placeCMD).ToPtr();
}
