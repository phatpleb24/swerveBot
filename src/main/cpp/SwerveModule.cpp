

#include "SwerveModule.h"
#include "Conversions.h"

SwerveModule::SwerveModule(int driveChannel, int turnChannel, units::volt_t kSAngular, units::unit_t<kvA_unit> kVAngular) : driveMotor(driveChannel), turnMotor(turnChannel), turnFeedforward(kSAngular, kVAngular)
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
    return {Conversions::NativeUnitsToDistanceMeters(driveMotor.GetSelectedSensorPosition(), SwerveConstants::kDriveGearRatio, SwerveConstants::kWheelRadiusInches), Conversions::NativeUnitsToDegrees(turnMotor.GetSelectedSensorPosition(), SwerveConstants::kAngleGearRatio)};
}

frc::SwerveModuleState SwerveModule::getState()
{
    return {Conversions::NativeUnitstoVelocityMPS(driveMotor.GetSelectedSensorVelocity(), SwerveConstants::kDriveGearRatio, SwerveConstants::kWheelRadiusInches), Conversions::NativeUnitsToDegrees(turnMotor.GetSelectedSensorPosition(), SwerveConstants::kAngleGearRatio)};
}

void SwerveModule::setDesiredState(const frc::SwerveModuleState& referenceState)
{
    auto state = frc::SwerveModuleState::Optimize(referenceState, Conversions::NativeUnitsToDegrees(turnMotor.GetSelectedSensorPosition(), SwerveConstants::kAngleGearRatio));
    auto drivePIDOutput = drivePID.Calculate(Conversions::NativeUnitstoVelocityMPS(driveMotor.GetSelectedSensorVelocity(), SwerveConstants::kDriveGearRatio, SwerveConstants::kWheelRadiusInches).value(), state.speed.value());
    auto driveFeedForwardOutput = driveFeedforward.Calculate(state.speed);
    auto turnPIDOutput = turnPID.Calculate(Conversions::NativeUnitsToDegrees(turnMotor.GetSelectedSensorPosition(), SwerveConstants::kAngleGearRatio), state.angle.Radians());
    auto turnFeedForwardOutput = turnFeedforward.Calculate(turnPID.GetSetpoint().velocity);

    driveMotor.SetVoltage(units::volt_t{drivePIDOutput} + driveFeedForwardOutput);
    turnMotor.SetVoltage(units::volt_t{turnPIDOutput} + turnFeedForwardOutput);
}

void SwerveModule::resetEncoder()
{
    driveMotor.SetSelectedSensorPosition(0);
}

double SwerveModule::getTurnEncoderCnt()
{
    return turnMotor.GetSelectedSensorPosition();
}

double SwerveModule::getDegrees()
{
    return Conversions::NativeUnitsToDegrees(getTurnEncoderCnt(), SwerveConstants::kAngleGearRatio).value();
}