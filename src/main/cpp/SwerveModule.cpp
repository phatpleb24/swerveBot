#include "SwerveModule.h"
#include "Conversions.h"

SwerveModule::SwerveModule(int driveChannel, int turnChannel) : driveMotor(driveChannel), turnMotor(turnChannel)
{
    turnPID.EnableContinuousInput(-units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
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

}

frc::SwerveModulePosition SwerveModule::getPosition()
{
    return {Conversions::NativeUnitsToDistanceMeters(driveMotor.GetSelectedSensorPosition(), SwerveConstants::kGearRatio, SwerveConstants::kWheelRadiusInches), Conversions::NativeUnitsToDegrees(turnMotor.GetSelectedSensorPosition(), SwerveConstants::kGearRatio)};
}

frc::SwerveModuleState SwerveModule::getState()
{
    return {Conversions::NativeUnitstoVelocityMPS(driveMotor.GetSelectedSensorVelocity(), SwerveConstants::kGearRatio, SwerveConstants::kWheelRadiusInches), Conversions::NativeUnitsToDegrees(turnMotor.GetSelectedSensorPosition(), SwerveConstants::kGearRatio)};
}

void SwerveModule::setDesiredState(const frc::SwerveModuleState& referenceState)
{
    auto state = frc::SwerveModuleState::Optimize(referenceState, Conversions::NativeUnitsToDegrees(turnMotor.GetSelectedSensorPosition(), SwerveConstants::kGearRatio));
    auto drivePIDOutput = drivePID.Calculate(Conversions::NativeUnitstoVelocityMPS(driveMotor.GetSelectedSensorVelocity(), SwerveConstants::kGearRatio, SwerveConstants::kWheelRadiusInches).value(), state.speed.value());
    auto driveFeedForwardOutput = driveFeedforward.Calculate(state.speed);
    auto turnPIDOutput = turnPID.Calculate(Conversions::NativeUnitsToDegrees(turnMotor.GetSelectedSensorPosition(), SwerveConstants::kGearRatio), state.angle.Radians());
    auto turnFeedForwardOutput = turnFeedforward.Calculate(turnPID.GetSetpoint().velocity);

    driveMotor.SetVoltage(units::volt_t{drivePIDOutput} + driveFeedForwardOutput);
    turnMotor.SetVoltage(units::volt_t{turnPIDOutput} + turnFeedForwardOutput);
}

void SwerveModule::resetEncoder()
{
    driveMotor.SetSelectedSensorPosition(0);
}