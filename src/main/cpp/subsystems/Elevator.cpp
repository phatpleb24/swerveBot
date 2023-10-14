#include "subsystems/Elevator.h"
#include "Conversions.h"
#include <Eigen/Core>
#include <frc/EigenCore.h>
#include <frc/smartdashboard/SmartDashboard.h>

Elevator::Elevator()
{
    leftMotor.ConfigFactoryDefault();
    rightMotor.ConfigFactoryDefault();
    //rightMotor.Follow(leftMotor);
    rightMotor.Follow(leftMotor);
    leftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    leftMotor.SetSelectedSensorPosition(0);
    leftMotor.Config_kP(0,0);
    leftMotor.Config_kF(0,0);
    leftMotor.Config_kD(0,0);
    leftMotor.Config_kI(0,0);
    rightMotor.SetNeutralMode(Coast);
    rightMotor.SetNeutralMode(Coast);
    leftMotor.SetInverted(TalonFXInvertType::Clockwise);
    rightMotor.SetInverted(InvertType::OpposeMaster);
    leftMotor.ConfigMotionCruiseVelocity(1500);
    leftMotor.ConfigMotionAcceleration(100);
    leftMotor.SelectProfileSlot(0,0);
}

void Elevator::setState()
{
    //leftMotor.Set(ControlMode::MotionMagic, Conversions::DistanceToNativeUnits(setpoint, kGearRatio, kDrumRadius), DemandType_ArbitraryFeedForward, gravityFF);
}

void Elevator::setPos(units::meter_t x)
{
    setpoint = x;
}

void Elevator::Periodic()
{
    frc::SmartDashboard::PutNumber("Elevator Pos", leftMotor.GetSelectedSensorPosition());
}

void Elevator::voltage()
{
    leftMotor.SetVoltage(4_V);
}