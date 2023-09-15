#pragma once

#include <numbers>
#include <units/length.h>
#include <frc/geometry/Rotation2d.h>
#include "Constants.h"

#define sc SwerveConstants

namespace Conversions
{
    using namespace sc;
    static units::degree_t NativeUnitsToDegrees(double nativeUnits, double gearRatio)
    {
        return units::degree_t{nativeUnits/kUnitsPerRevolution / gearRatio * 360.0};
    }

    static double DegreesToNativeUnits(units::degree_t position, double gearRatio)
    {
        return position.value() / (360.0 / (gearRatio * 2048.0));
    }

    static int DistanceToNativeUnits(units::meter_t position, double gearRatio, units::inch_t wheelRadiusInches)
    {
        double wheelRotation = position / (2 * std::numbers::pi * wheelRadiusInches);
        double motorRotations = wheelRotation * gearRatio;
        int sensorCounts = (int)(motorRotations * kUnitsPerRevolution);
        return sensorCounts;
    }

    static int VelocityToNativeUnits(units::meters_per_second_t velocity, units::inch_t wheelRadiusInches, double gearRatio)
    {
        auto wheelRotationPerSecond = velocity / (2 * std::numbers::pi * wheelRadiusInches);
        auto motorRotationPerSecond = wheelRotationPerSecond * gearRatio;
        double motorRotationPer100ms = motorRotationPerSecond * 1_s / k100msPerSecond;
        int sensorCountPer100ms = (int)(motorRotationPer100ms * kUnitsPerRevolution);
        return sensorCountPer100ms;
    }

    static units::meter_t NativeUnitsToDistanceMeters(double sensorCounts, double gearRatio, units::inch_t wheelRadiusInches)
    {
        double motorRotations = (double)sensorCounts/kUnitsPerRevolution;
        double wheelRotations = motorRotations / gearRatio;
        units::meter_t position = wheelRotations * (2 * std::numbers::pi * wheelRadiusInches);
        return position;
    }

    static units::meters_per_second_t NativeUnitstoVelocityMPS(double sensorCounts, double gearRatio, units::inch_t wheelRadiusInches)
    {
        double motorRotationsPer100ms = sensorCounts / kUnitsPerRevolution;
        auto motorRotationPerSecond = motorRotationsPer100ms / (1_s/k100msPerSecond);
        auto wheelRotationPerSecond = motorRotationPerSecond / gearRatio;
        auto velocity = wheelRotationPerSecond * (2 * std::numbers::pi * wheelRadiusInches);
        return velocity;
    }
    static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }
}