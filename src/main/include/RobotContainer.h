// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  Drivetrain swerve;
  Elevator elevator;
  Intake intake;
  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

 private:
  void ConfigureBindings();
  frc2::CommandXboxController controller{0};
  frc2::CommandXboxController controller2{1};
};
