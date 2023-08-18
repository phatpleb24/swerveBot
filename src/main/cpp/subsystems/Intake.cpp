#include "subsystems/Intake.h"

Intake::Intake()
{
    intakeMotor.ConfigFactoryDefault();
    wristMotor.ConfigFactoryDefault();
    wristMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    wristMotor.SetSelectedSensorPosition(0);

}

void Intake::intakeSpin(double x)
{
    intakeMotor.SetVoltage(units::volt_t{x});
}

void Intake::wristSetPoint(units::degree_t x)
{
    setPoint = x;
}

void Intake::wristSpin(double x)
{
    wristMotor.SetVoltage(units::volt_t{x});
}

units::degree_t Intake::getPoint()
{
    return setPoint;
}

void Intake::Periodic()
{
    double PIDout = wristPID.Calculate(Conversions::NativeUnitsToDegrees(wristMotor.GetSelectedSensorPosition(), gearRatio).value(), setPoint.value());
    units::volt_t feedforwardOut = feedforward.Calculate(setPoint, 0_rad / 1_s);
    //wristMotor.SetVoltage(units::volt_t{PIDout} + feedforwardOut);
}