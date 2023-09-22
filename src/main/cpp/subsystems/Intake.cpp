#include "subsystems/Intake.h"

Intake::Intake()
{
    intakeMotor.ConfigFactoryDefault();
    wristMotor.ConfigFactoryDefault();
    wristMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    wristMotor.SetSelectedSensorPosition(0);
    wristMotor.SetInverted(false);
    wristMotor.SetNeutralMode(Brake);
    wristMotor.Config_kD(0, 0);
    wristMotor.Config_kF(0, 0.05);
    wristMotor.Config_kI(0, 0);
    wristMotor.Config_kP(0, 0.3);
    wristMotor.ConfigMotionCruiseVelocity(100);
    wristMotor.ConfigMotionAcceleration(30);
    //setPoint = 30_deg;
}

void Intake::intakeSpin(double x)
{
    intakeMotor.SetVoltage(units::volt_t{x});
}

void Intake::wristSpin(double x)
{
    wristMotor.SetVoltage(units::volt_t{x});
}

units::degree_t Intake::getPoint()
{
    return setPoint;
}

void Intake::setSetpoint(units::degree_t x)
{
    setPoint = x;
}

void Intake::Periodic()
{
    /*double PIDout = wristPID.Calculate(Conversions::NativeUnitsToDegrees(wristMotor.GetSelectedSensorPosition(), gearRatio).value(), setPoint.value());
    units::volt_t feedforwardOut = feedforward.Calculate(setPoint, 0_rad / 1_s);
    wristMotor.SetVoltage(units::volt_t{PIDout} + feedforwardOut);
    printf("Setpoint: %f\n", setPoint.value());
    printf("Wrist Voltage: %f\n", (units::volt_t{PIDout} + feedforwardOut).value());*/
}

void Intake::moveWrist()
{
    wristMotor.Set(ControlMode::MotionMagic, Conversions::DegreesToNativeUnits(setPoint, gearRatio));
}