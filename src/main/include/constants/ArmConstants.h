#pragma once
#include <units/angular_velocity.h>
#include <subzero/constants/ColorConstants.h>
#include <subzero/singleaxis/ISingleAxisSubsystem.h>
#include <units/length.h>
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//ALL VALUES ARE CURRENTLY PLACE HOLDERS FOR KNOW!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



namespace AlgaeArmConstants{
    constexpr int kArmMotorId = -1;
    constexpr double kArmP = -1;
    constexpr double kArmI = -1;
    constexpr double kArmD = -1;
    constexpr double kArmIZone = -1;
    constexpr double kArmFF = -1;
    
    constexpr units::revolutions_per_minute_t kMaxRpm = 1_rpm;
    constexpr units::degree_t kHomeRotation = 10_deg;
    constexpr units::degree_t kMaxRotation = 190_deg;
    constexpr units::degree_t kArmRelativeDistancePerRev = 360_deg * (1 / 8.75);
    constexpr units::degree_t kArmAbsoluteDistancePerRev = 360_deg;
    constexpr units::degrees_per_second_t kDefaultVelocity = 10_deg_per_s;
    constexpr double kVelocityScalar = 1.0;
    constexpr units::degree_t kTolerance = 2_deg;
    constexpr units::meter_t kArmLength = 0.2_m;
    
    static const subzero::SingleAxisMechanism kArmMechanism = {
    // length
    0.2_m,
    // min angle
    110_deg,
    // line width
    6,
    // color
    subzero::ColorConstants::kBlue};

    const frc::TrapezoidProfile<units::degree>::Constraints
        kRotationalAxisConstraints{360_deg_per_s * 2.2, 360_deg_per_s_sq * 2.2};

}

