// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <units/angle.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();

  swerve.SetDefaultCommand(frc2::RunCommand(
    [this]
    {
      if(controller.GetLeftStickButtonPressed()) swerve.fieldRelative = !swerve.fieldRelative;
      auto xSpeed = swerve.m_xspeedLimiter.Calculate(std::clamp(frc::ApplyDeadband(controller.GetLeftY(), 0.05),-0.2, 0.2) /** swerve.kMaxSpeed.value()*/);
      auto ySpeed = swerve.m_yspeedLimiter.Calculate(std::clamp(frc::ApplyDeadband(controller.GetLeftX(), 0.05), -0.2,0.2) /** swerve.kMaxSpeed.value()*/);
      auto rotSpeed = swerve.m_rotLimiter.Calculate(std::clamp(frc::ApplyDeadband(controller.GetRightX(), 0.05),-0.2,  0.2) /** swerve.kMaxAngularSpeed.value()*/);
      swerve.Drive(xSpeed * swerve.kMaxSpeed, ySpeed * swerve.kMaxSpeed, rotSpeed * swerve.kMaxAngularSpeed, swerve.fieldRelative);
    }, 
    {&swerve}));
  
  elevator.SetDefaultCommand(frc2::RunCommand(
    [this]
    {
      if(controller.GetAButton())
      {
        elevator.setState(Elevator::kRaisedPosition);
      }
      else if (controller.GetBButton())
      {
        elevator.setState(Elevator::kLoweredPosition);
      }
    }, {&elevator}
  ));

  intake.SetDefaultCommand(frc2::RunCommand(
    [this]
    {
      if(controller.GetYButton())
      {
        intake.moveWrist(60_deg);
      }
      else if(controller.GetXButton()){
        intake.moveWrist(0_deg);
      }

      if(controller.GetLeftBumper()){
        intake.intakeSpin(3);
      }
      else if(controller.GetRightBumper()){
        intake.intakeSpin(-3);
      }
      else{
        intake.intakeSpin(0);
      }
    }, {&intake}
  ));
}

void RobotContainer::ConfigureBindings() 
{
 
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
