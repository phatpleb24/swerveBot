#include "subsystems/Elevator.h"
#include "Conversions.h"
#include <Eigen/Core>
#include <frc/EigenCore.h>

Elevator::Elevator()
{
    leftMotor.ConfigFactoryDefault();
    rightMotor.ConfigFactoryDefault();
    rightMotor.Follow(leftMotor);
    leftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    leftMotor.SetSelectedSensorPosition(0);
    frc::SmartDashboard::PutData("Elevator", &mech2d);
}

void Elevator::SimulationPeriodic()
{
    motorSim.SetBusVoltage(frc::RobotController::GetInputVoltage());
    elevatorSim.SetInput(frc::Vectord<1>{motorSim.GetMotorOutputLeadVoltage()});
    elevatorSim.Update(20_ms);
    motorSim.SetIntegratedSensorRawPosition(Conversions::DistanceToNativeUnits(elevatorSim.GetPosition(), kGearRatio, kDrumRadius));
    motorSim.SetIntegratedSensorVelocity(Conversions::VelocityToNativeUnits(elevatorSim.GetVelocity(), kDrumRadius, kGearRatio));
    elevatorMech2d->SetLength(units::inch_t{elevatorSim.GetPosition()}.value());
}

void Elevator::Periodic()
{
    loop.SetNextR(frc::Vectord<2>{lastProfiledReference.position.value(), lastProfiledReference.velocity.value()});
    loop.Correct(frc::Vectord<1>{Conversions::NativeUnitsToDistanceMeters(leftMotor.GetSelectedSensorPosition(), kGearRatio, kDrumRadius).value()});
    loop.Predict(20_ms);
    leftMotor.SetVoltage(units::volt_t{loop.U(0)});
}

void Elevator::reset()
{
    loop.Reset(frc::Vectord<2>{Conversions::NativeUnitsToDistanceMeters(leftMotor.GetSelectedSensorPosition(), kGearRatio, kDrumRadius).value(), Conversions::NativeUnitstoVelocityMPS(leftMotor.GetSelectedSensorVelocity(), kGearRatio, kDrumRadius).value()});
    lastProfiledReference = {Conversions::NativeUnitsToDistanceMeters(leftMotor.GetSelectedSensorPosition(), kGearRatio, kDrumRadius), Conversions::NativeUnitstoVelocityMPS(leftMotor.GetSelectedSensorVelocity(), kGearRatio, kDrumRadius)};
}

void Elevator::setState(units::meter_t goalPoint)
{
    frc::TrapezoidProfile<units::meters>::State goal;
    goal = {goalPoint, 0_fps};
    lastProfiledReference = (frc::TrapezoidProfile<units::meters>(constraints, goal, lastProfiledReference)).Calculate(20_ms);
}
