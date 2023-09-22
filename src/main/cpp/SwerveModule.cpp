#include "SwerveModule.h"
#include "Conversions.h"

SwerveModule::SwerveModule(int driveChannel, int turnChannel, int encoderChannel, int offset, bool invert) 
: driveMotor(driveChannel)
, turnMotor(turnChannel)
, angleEncoder(encoderChannel)
{
    angleOffset = offset;
    driveMotor.ConfigFactoryDefault();
    turnMotor.ConfigFactoryDefault();

  /*m_rightFrontMotor.SetInverted(DriveConstants::kRightDirection);
  m_rightFollowerMotor.SetInverted(DriveConstants::kRightDirection);
  m_leftFrontMotor.SetInverted(DriveConstants::kLeftDirection);
  m_leftFollowerMotor.SetInverted(DriveConstants::kLeftDirection);*/

    driveMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    driveMotor.SetSelectedSensorPosition(0);
    turnMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    turnMotor.SetSelectedSensorPosition(0);

    driveMotor.SetInverted(invert);
    turnMotor.SetInverted(TalonFXInvertType::Clockwise);

    turnMotor.ConfigNominalOutputForward(0);
    turnMotor.ConfigNominalOutputReverse(0);
    turnMotor.ConfigPeakOutputForward(1);
    turnMotor.ConfigPeakOutputReverse(-1);
    turnMotor.SetNeutralMode(Brake);
    turnMotor.Config_kP(0, 0.3);
    turnMotor.Config_kD(0, 0);
    turnMotor.Config_kI(0, 0);
    turnMotor.Config_kF(0,0);
    turnMotor.ConfigNeutralDeadband(0.05);
    turnMotor.SelectProfileSlot(0,0);
    turnMotor.SetStatusFramePeriod(StatusFrame::Status_1_General_, 99);
    turnMotor.SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 15);

    angleEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange(Unsigned_0_to_360));
    angleEncoder.ConfigSensorDirection(true);
    angleEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy(BootToAbsolutePosition));

    resetToAbsolute();
}

frc::SwerveModulePosition SwerveModule::getPosition()
{
    return {Conversions::NativeUnitsToDistanceMeters(driveMotor.GetSelectedSensorPosition(), SwerveConstants::kDriveGearRatio, SwerveConstants::kWheelRadiusInches), Conversions::NativeUnitsToDegrees(turnMotor.GetSelectedSensorPosition(), SwerveConstants::kAngleGearRatio)};
}

frc::SwerveModuleState SwerveModule::getState()
{
    return {Conversions::NativeUnitstoVelocityMPS(driveMotor.GetSelectedSensorVelocity(), SwerveConstants::kDriveGearRatio, SwerveConstants::kWheelRadiusInches), Conversions::NativeUnitsToDegrees(turnMotor.GetSelectedSensorPosition(), SwerveConstants::kAngleGearRatio)};
}

void SwerveModule::setDesiredState(const frc::SwerveModuleState& referenceState)
{
    auto state = frc::SwerveModuleState::Optimize(referenceState, Conversions::NativeUnitsToDegrees(turnMotor.GetSelectedSensorPosition(), SwerveConstants::kAngleGearRatio));
    auto drivePIDOutput = drivePID.Calculate(
        Conversions::NativeUnitstoVelocityMPS(driveMotor.GetSelectedSensorVelocity(), 
        SwerveConstants::kDriveGearRatio, SwerveConstants::kWheelRadiusInches).value(), state.speed.value());
    auto driveFeedForwardOutput = driveFeedforward.Calculate(state.speed);
    //auto turnPIDOutput = turnPID.Calculate(Conversions::NativeUnitsToDegrees(turnMotor.GetSelectedSensorPosition(), SwerveConstants::kAngleGearRatio), state.angle.Radians());
    //auto turnFeedForwardOutput = turnFeedforward.Calculate(turnPID.GetSetpoint().velocity);

    driveMotor.SetVoltage(units::volt_t{drivePIDOutput} + driveFeedForwardOutput);
    turnMotor.Set(TalonFXControlMode::Position, Conversions::DegreesToNativeUnits(state.angle.Degrees(), SwerveConstants::kAngleGearRatio));
    
}

void SwerveModule::resetEncoder()
{
    driveMotor.SetSelectedSensorPosition(0);
}

double SwerveModule::getCanCoder(){
    return angleEncoder.GetAbsolutePosition();
}

void SwerveModule::resetToAbsolute(){
    double absolutePosition = Conversions::DegreesToNativeUnits(units::degree_t{getCanCoder() - angleOffset}, SwerveConstants::kAngleGearRatio);
    turnMotor.SetSelectedSensorPosition(absolutePosition);
}

void SwerveModule::configAngleEncoder(){        
    angleEncoder.ConfigFactoryDefault();
    angleEncoder.ConfigAllSettings(CANCoderConfig);
}

double SwerveModule::getTurnEncoderCnt()
{
    return turnMotor.GetSelectedSensorPosition();
}

double SwerveModule::getDegrees()
{
    return Conversions::NativeUnitsToDegrees(getTurnEncoderCnt(), SwerveConstants::kAngleGearRatio).value();
}
