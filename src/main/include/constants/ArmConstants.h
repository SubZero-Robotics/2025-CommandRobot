#pragma once
#include <units/angular_velocity.h>
#include <subzero/constants/ColorConstants.h>
#include <subzero/singleaxis/ISingleAxisSubsystem.h>
#include <units/length.h>
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//ALL VALUES ARE CURRENTLY PLACE HOLDERS FOR KNOW!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



namespace AlgaeArmConstants{
    constexpr int kArmMotorId = 62;
    constexpr int kIntakeMotorId = 20;
    constexpr double kP = 0.075;
    constexpr double kI = 0.0;
    constexpr double kD = 0.0;
    constexpr double kIZone = 0.1;
    constexpr double kFF = 0.1;
    
    constexpr units::revolutions_per_minute_t kMaxRpm = 1_rpm;
    constexpr units::degree_t kHomeRotation = 10_deg;
    constexpr units::degree_t kMaxRotation = 190_deg;
    constexpr units::degree_t kRelativeDistancePerRev = 360_deg * (1 / 8.75);
    constexpr units::degree_t kAbsoluteDistancePerRev = 360_deg;
    constexpr units::degrees_per_second_t kDefaultVelocity = 10_deg_per_s;
    constexpr double kVelocityScalar = 1.0;
    constexpr units::degree_t kTolerance = 2_deg;
    constexpr units::meter_t kArmLength = 0.2_m;
    
    static const subzero::SingleAxisMechanism kAlgaeArmMechanism = {
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

namespace CoralArmConstants{
    constexpr int kArmMotorId = 20;
    constexpr int kIntakeMotorId = 20;
    constexpr double kP = 0.1;
    constexpr double kI = 0.1;
    constexpr double kD = 0.1;
    constexpr double kIZone = 0.1;
    constexpr double kFF = 0.1;
    
    constexpr units::revolutions_per_minute_t kMaxRpm = 1_rpm;
    constexpr units::degree_t kHomeRotation = 10_deg;
    constexpr units::degree_t kMaxRotation = 190_deg;
    constexpr units::degree_t kRelativeDistancePerRev = 360_deg * (1 / 8.75);
    constexpr units::degree_t kAbsoluteDistancePerRev = 360_deg;
    constexpr units::degrees_per_second_t kDefaultVelocity = 10_deg_per_s;
    constexpr double kVelocityScalar = 1.0;
    constexpr units::degree_t kTolerance = 2_deg;
    constexpr units::meter_t kArmLength = 0.2_m;
    
    static const subzero::SingleAxisMechanism kCoralArmMechanism = {
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

