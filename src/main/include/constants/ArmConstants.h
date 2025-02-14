#pragma once
#include <units/angular_velocity.h>
#include <subzero/constants/ColorConstants.h>
#include <subzero/singleaxis/ISingleAxisSubsystem.h>
#include <units/length.h>
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//ALL VALUES ARE CURRENTLY PLACE HOLDERS FOR KNOW!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



namespace AlgaeArmConstants{
    constexpr int kArmMotorId = 17;
    constexpr int kIntakeMotorId = 6;
    constexpr double kP = 0.2;
    constexpr double kI = 0.0;
    constexpr double kD = 0.001;
    constexpr double kIZone = 0.0;
    constexpr double kFF = 0.0;
    
    constexpr units::revolutions_per_minute_t kMaxRpm = 5676_rpm;
    constexpr units::degree_t kHomeRotation = 0_deg;
    constexpr units::degree_t kMaxRotation = 85_deg;
    constexpr units::degree_t kRelativeDistancePerRev = 360_deg / 75;
    constexpr units::degree_t kAbsoluteDistancePerRev = 360_deg;
    constexpr units::degrees_per_second_t kDefaultVelocity = 10_deg_per_s;
    constexpr double kVelocityScalar = 1.0;
    constexpr units::degree_t kTolerance = 2_deg;
    constexpr units::meter_t kArmLength = 17_in;
    
    static const subzero::SingleAxisMechanism kAlgaeArmMechanism = {
    // length
    0.2_m,
    // min angle
    110_deg,
    // line width
    6,
    // color
    subzero::ColorConstants::kBlue};

    // const frc::TrapezoidProfile<units::degree>::Constraints
    //     kRotationalAxisConstraints{360_deg_per_s * 2.2, 360_deg_per_s_sq * 2.2};

    const frc::TrapezoidProfile<units::degree>::Constraints
        kRotationalAxisConstraints;

}

namespace CoralArmConstants{
    constexpr int kArmMotorId = 16;
    constexpr int kIntakeMotorId = 15;
    constexpr double kP = 0.25;
    constexpr double kI = 0.0;
    constexpr double kD = 0.0;
    constexpr double kIZone = 0.0;
    constexpr double kFF = 0.0;
    
    constexpr units::revolutions_per_minute_t kMaxRpm = 1_rpm;
    constexpr units::degree_t kHomeRotation = 5_deg;
    constexpr units::degree_t kMaxRotation = 180_deg;
    constexpr units::degree_t kRelativeDistancePerRev = 360_deg / (75 * 4.7); // 4.7 is the ratio of the chain gear
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

