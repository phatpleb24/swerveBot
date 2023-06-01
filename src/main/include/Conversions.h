#pragma once

#include <numbers>
#include <units/length.h>
#include "Constants.h"

#define sc SwerveConstants

namespace Conversions
{
    using namespace sc;
    units::degree_t NativeUnitsToDegrees(double nativeUnits, double gearRatio)
    {
        return units::degree_t{nativeUnits * (360.0 / (gearRatio * kUnitsPerRevolution))};
    }

    double DegreesToNativeUnits(units::degree_t position, double gearRatio)
    {
        return position.value() / (360.0 / (gearRatio * 2048.0));
    }

    int DistanceToNativeUnits(units::meter_t position, double gearRatio, units::inch_t wheelRadiusInches)
    {
        double wheelRotation = position / (2 * std::numbers::pi * wheelRadiusInches);
        double motorRotations = wheelRotation * gearRatio;
        int sensorCounts = (int)(motorRotations * kUnitsPerRevolution);
        return sensorCounts;
    }

    int VelocityToNativeUnits(units::meters_per_second_t velocity, units::inch_t wheelRadiusInches, double gearRatio)
    {
        auto wheelRotationPerSecond = velocity / (2 * std::numbers::pi * wheelRadiusInches);
        auto motorRotationPerSecond = wheelRotationPerSecond * gearRatio;
        double motorRotationPer100ms = motorRotationPerSecond * 1_s / k100msPerSecond;
        int sensorCountPer100ms = (int)(motorRotationPer100ms * kUnitsPerRevolution);
        return sensorCountPer100ms;
    }

    units::meter_t NativeUnitsToDistanceMeters(double sensorCounts, double gearRatio, units::inch_t wheelRadiusInches)
    {
        double motorRotations = (double)sensorCounts/kUnitsPerRevolution;
        double wheelRotations = motorRotations / gearRatio;
        units::meter_t position = wheelRotations * (2 * std::numbers::pi * wheelRadiusInches);
        return position;
    }

    units::meters_per_second_t NativeUnitstoVelocityMPS(double sensorCounts, double gearRatio, units::inch_t wheelRadiusInches)
    {
        double motorRotationsPer100ms = sensorCounts / kUnitsPerRevolution;
        auto motorRotationPerSecond = motorRotationsPer100ms / (1_s/k100msPerSecond);
        auto wheelRotationPerSecond = motorRotationPerSecond / gearRatio;
        auto velocity = wheelRotationPerSecond * (2 * std::numbers::pi * wheelRadiusInches);
        return velocity;
    }

}