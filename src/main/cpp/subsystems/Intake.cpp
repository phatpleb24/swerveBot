#include "subsystems/Intake.h"

Intake::Intake()
{
    intakeMotor.ConfigFactoryDefault();
    wristMotor.ConfigFactoryDefault();
    wristMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    wristMotor.SetSelectedSensorPosition(Conversions::DegreesToNativeUnits(75_deg, gearRatio));
    wristMotor.SetInverted(false);
    wristMotor.SetNeutralMode(Brake);
    wristMotor.Config_kD(0, 0);
    wristMotor.Config_kF(0, 2.5);
    wristMotor.Config_kI(0, 0);
    wristMotor.Config_kP(0, 0.08);
    wristMotor.ConfigMotionCruiseVelocity(1000);
    wristMotor.ConfigMotionAcceleration(80);
    //setPoint = 30_deg;
}

double Intake::getEncoderValue(){
    return wristMotor.GetSelectedSensorPosition();
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
    frc::SmartDashboard::PutNumber("Intake state", intakeState);
}

void Intake::moveWrist()
{
    //wristMotor.Set(ControlMode::MotionMagic, Conversions::DegreesToNativeUnits(setPoint, gearRatio));
}