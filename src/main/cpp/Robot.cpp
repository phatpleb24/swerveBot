// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc2/command/CommandScheduler.h>
#include "commands/PlacementSequence.h"

void Robot::RobotInit() {
  SetNetworkTablesFlushEnabled(true);
  
  frc::DataLogManager::Start();

  chooser.AddOption("Blue Left", "Left");
  chooser.AddOption("Mid", "Mid");
  chooser.SetDefaultOption("Place", "Place");

  // chooser.SetDefaultOption("Place Only", place.get());
  // chooser.AddOption("Left", leftCMD.get());
  // chooser.AddOption("Right", rightCMD.get());
  // chooser.AddOption("Mid", midCMD.get());
  frc::SmartDashboard::PutData("Auto Modes", &chooser);
  m_container.swerve.wheelReset();
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  std::string autoMode = chooser.GetSelected();

  frc2::CommandPtr leftCMD = m_container.GetAutonomousCommand();
  frc2::CommandPtr midCMD = m_container.balanceRoutine();
  frc2::CommandPtr placeCMD = m_container.place();

  if(autoMode == "Left") m_autonomousCommand = std::move(leftCMD);
  else if(autoMode == "Mid") m_autonomousCommand = std::move(midCMD);
  else m_autonomousCommand = std::move(placeCMD);

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
