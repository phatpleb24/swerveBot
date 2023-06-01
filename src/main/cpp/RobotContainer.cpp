// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();

  swerve.SetDefaultCommand(frc2::RunCommand([this]
  {
      auto xSpeed = -swerve.m_xspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetLeftY(), 0.02) /** swerve.kMaxSpeed.value()*/);
      auto ySpeed = swerve.m_yspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetLeftX(), 0.02) /** swerve.kMaxSpeed.value()*/);
      auto rotSpeed = swerve.m_rotLimiter.Calculate(frc::ApplyDeadband(controller.GetRightX(), 0.02) /** swerve.kMaxAngularSpeed.value()*/);
      swerve.Drive(xSpeed * swerve.kMaxSpeed, ySpeed * swerve.kMaxSpeed, rotSpeed * swerve.kMaxAngularSpeed, true);
  }));
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
