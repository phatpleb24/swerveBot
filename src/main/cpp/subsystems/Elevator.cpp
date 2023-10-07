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
    leftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    leftMotor.SetSelectedSensorPosition(0);
    
    //Orig Code
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
    
    //my code
    //updateElevatorMeters();
    //frc::SmartDashboard::PutNumber("Elevator Setpoint", (double)elevatorSetpointMeters);
}

void Elevator::reset()
{
    loop.Reset(frc::Vectord<2>{Conversions::NativeUnitsToDistanceMeters(leftMotor.GetSelectedSensorPosition(), kGearRatio, kDrumRadius).value(), Conversions::NativeUnitstoVelocityMPS(leftMotor.GetSelectedSensorVelocity(), kGearRatio, kDrumRadius).value()});
    /* My code
    elevatorPID.Reset(Conversions::NativeUnitsToDistanceMeters(leftMotor.GetSelectedSensorPosition(), SwerveConstants::kAngleGearRatio, SwerveConstants::kWheelRadiusInches));*/
    lastProfiledReference = {Conversions::NativeUnitsToDistanceMeters(leftMotor.GetSelectedSensorPosition(), kGearRatio, kDrumRadius), Conversions::NativeUnitstoVelocityMPS(leftMotor.GetSelectedSensorVelocity(), kGearRatio, kDrumRadius)};
}

void Elevator::setState(units::meter_t goalPoint)
{
    /* My code
    Elevator::elevatorSetpointMeters = goalPoint;*/
   
}
/* Added Code
void Elevator::updateElevatorMeters(){
    // Takes in current elevator position in meters and the setpoint in meters and outputs change needed
    double calculated = elevatorPID.Calculate(Conversions::NativeUnitsToDistanceMeters(leftMotor.GetSelectedSensorPosition(), SwerveConstants::kDriveGearRatio, SwerveConstants::kWheelRadiusInches), elevatorSetpointMeters);

    // caculated = slew.calculate(caculated);
    
    // Set motors to need speed change
    leftMotor.Set(calculated);
}
*/
