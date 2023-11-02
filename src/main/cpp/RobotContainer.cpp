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
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>
#include "commands/IntakeDown.h"

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
      auto rotSpeed = swerve.m_rotLimiter.Calculate(std::clamp(frc::ApplyDeadband(controller.GetRightX(), 0.10),-1.0,  1.0) /** swerve.kMaxAngularSpeed.value()*/);
      swerve.Drive(xSpeed * SwerveConstants::kMaxSpeed, ySpeed * SwerveConstants::kMaxSpeed, rotSpeed * SwerveConstants::kMaxAngularSpeed, swerve.fieldRelative);
    }, 
    {&swerve}));
  
  elevator.SetDefaultCommand(frc2::RunCommand(
    [this]
    {
      if(controller2.GetAButtonPressed())
      {
        //elevator.voltage();
        //elevator.setPos(27_in);
      }
      else if(controller2.GetBButtonPressed())
      {
        elevator.setPos(0_m);
      }
      //elevator.setState();
      //elevator.raiseElevator();
    }, {&elevator}
  ));

  intake.SetDefaultCommand(frc2::RunCommand(
    [this]
    {
      if(controller2.GetYButton())
      {
        intake.setSetpoint(80_deg);
      }
      else if(controller2.GetXButton()){
        intake.setSetpoint(0_deg);
      }
      intake.moveWrist();
      if(controller2.GetLeftBumperPressed())
      {
        if (intake.intakeState != 1){
          intake.intakeState = 1;
        }
        else {
          intake.intakeState = 0;
        }
      }
      else if(controller2.GetRightBumperPressed())
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
  pathplanner::SwerveAutoBuilder swerveAuto
  {
    [this]() {return swerve.getPose();},
    [this](auto initPose) {swerve.resetOdometry(initPose);},
    pathplanner::PIDConstants(2,0,0),
    pathplanner::PIDConstants(2,0,0),
    [this](auto speeds) {swerve.driveFieldRelative(speeds);},
    eventMap,
    {&swerve},
    true
  };
  eventMap.emplace("intakeDown", std::make_shared<IntakeDown>(&intake));
  pathplanner::PathPlannerTrajectory traj1 = pathplanner::PathPlanner::loadPath("farLeftBlue", pathplanner::PathConstraints(4_mps, 3_mps_sq));
  pathplanner::PathPlannerTrajectory traj2 = pathplanner::PathPlanner::loadPath("farLeftBlueReturn", pathplanner::PathConstraints(4_mps, 3_mps_sq));
  auto moveCMD = swerveAuto.followPathWithEvents(traj1);
  auto returnCMD = swerveAuto.followPath(traj2);
  PlacementSequence placeCMD = PlacementSequence(&intake);
  return std::move(placeCMD).ToPtr()
    //.AndThen(std::move(frc2::InstantCommand([this] {swerve.wheelReset();}, {&swerve}).ToPtr()))
    .AndThen(std::move(moveCMD))
    .AndThen(std::move(frc2::InstantCommand([this] {intake.intakeSpin(0); intake.setSetpoint(75_deg);}, {&intake}).ToPtr()))
    .AndThen(std::move(returnCMD)).AndThen(std::move(placeCMD).ToPtr());
  //return std::move(placeCMD).ToPtr().AndThen(([this] {swerve.Drive(-0.5_mps, 0_mps, 0_rad_per_s, true);}, {&swerve}).Until(swerve.getPose().X() > 1.5_m));
}

frc2::CommandPtr RobotContainer::balanceRoutine()
{
  pathplanner::SwerveAutoBuilder swerveAuto
  {
    [this]() {return swerve.getPose();},
    [this](auto initPose) {swerve.resetOdometry(initPose);},
    pathplanner::PIDConstants(0,0,0),
    pathplanner::PIDConstants(0,0,0),
    [this](auto speeds) {swerve.driveFieldRelative(speeds);},
    eventMap,
    {&swerve},
    true
  };
  pathplanner::PathPlannerTrajectory traj1 = pathplanner::PathPlanner::loadPath("mid", pathplanner::PathConstraints(4_mps, 2_mps_sq));
  auto moveCMD = swerveAuto.followPath(traj1);
  PlacementSequence placeCMD = PlacementSequence(&intake);
  Balance balanceCMD = Balance(&swerve);
  printf("Init Pose: %.2f\n", traj1.getInitialPose().Rotation().Degrees());
  return std::move(placeCMD).ToPtr()
    .AndThen(std::move(moveCMD))
    .AndThen(std::move(balanceCMD).ToPtr());
}

frc2::CommandPtr RobotContainer::place()
{
  PlacementSequence placeCMD = PlacementSequence(&intake);
  return std::move(placeCMD).ToPtr();
}