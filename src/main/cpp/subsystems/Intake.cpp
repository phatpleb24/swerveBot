#include "subsystems/Intake.h"
#include "frc/smartdashboard/SmartDashboard.h"

Intake::Intake()
{
    intakeMotor.ConfigFactoryDefault();
    wristMotor.ConfigFactoryDefault();
    wristMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    wristMotor.SetSelectedSensorPosition(0);
    wristMotor.SetInverted(false);
    wristMotor.SetNeutralMode(Brake);
    wristMotor.Config_kD(0, 0);
    wristMotor.Config_kF(0, 5);
    wristMotor.Config_kI(0, 0);
    wristMotor.Config_kP(0, 2.61);
    wristMotor.SelectProfileSlot(0,0);
    wristMotor.ConfigMotionCruiseVelocity(4000);
    wristMotor.ConfigMotionAcceleration(300);
    
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
    frc::SmartDashboard::PutNumber("Arm Pos", Conversions::NativeUnitsToDegrees(wristMotor.GetSelectedSensorPosition(), gearRatio).value());
    frc::SmartDashboard::PutNumber("Arm Speed", wristMotor.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Setpoint", setPoint.value());
    frc::SmartDashboard::PutNumber("Arm Out", wristMotor.GetMotorOutputVoltage());
}

void Intake::moveWrist(units::degree_t x)
{
    //printf("Wrist moves\n");
    units::radian_t posRadian = Conversions::NativeUnitsToDegrees(wristMotor.GetSelectedSensorPosition(), gearRatio);
    double ff = cos(posRadian.value()) * maxGravityFF;
    printf("%f\n", ff);
    wristMotor.Set(ControlMode::MotionMagic, Conversions::DegreesToNativeUnits(x, gearRatio), DemandType_ArbitraryFeedForward, ff);
}