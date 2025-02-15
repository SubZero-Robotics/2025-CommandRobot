// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <subzero/motor/PidMotorController.h>
#include <subzero/constants/ColorConstants.h>
#include <subzero/singleaxis/ISingleAxisSubsystem.h>

#include <numbers>
#include <string>

#pragma once

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <subzero/singleaxis/ISingleAxisSubsystem.h>
#include <subzero/motor/PidMotorController.h>
#include <subzero/constants/ColorConstants.h>

#include <numbers>
#include <string>

using SparkMaxPidController =
    subzero::PidMotorController<rev::spark::SparkMax, rev::spark::SparkClosedLoopController,
                                rev::spark::SparkRelativeEncoder,
                                rev::spark::SparkAbsoluteEncoder, rev::spark::SparkMaxConfig>;
typedef
    subzero::PidMotorController<rev::spark::SparkFlex, rev::spark::SparkClosedLoopController,
                                rev::spark::SparkRelativeEncoder,
                                rev::spark::SparkAbsoluteEncoder, rev::spark::SparkFlexConfig> SparkFlexController;

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants



namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds

// Test sewrve
// constexpr units::meters_per_second_t kMaxSpeed = 4.8_mps;

// Season robot
constexpr units::meters_per_second_t kMaxSpeed = 4.92_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2 * std::numbers::pi};

const int kPigeonCanId = 13;

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration
constexpr units::meter_t kTrackWidth =
    24_in;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    24_in;  // Distance between centers of front and back wheels on robot

// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;

// SPARK MAX CAN IDs
constexpr int kFrontLeftDrivingCanId = 18;
constexpr int kRearLeftDrivingCanId = 1;
constexpr int kFrontRightDrivingCanId = 4;
constexpr int kRearRightDrivingCanId = 9;

constexpr int kFrontLeftTurningCanId = 2;
constexpr int kRearLeftTurningCanId = 20;
constexpr int kFrontRightTurningCanId = 5;
constexpr int kRearRightTurningCanId = 8;
}  // namespace DriveConstants

namespace ModuleConstants {
// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 12;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 21 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion

// Test swerve bot
// constexpr double kDrivingMotorReduction =
//     (45.0 * 21) / (kDrivingMotorPinionTeeth * 15);

constexpr double kDrivingMotorReduction =
    (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15);

constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

const std::string kOutAuto = "Out Auto";
const std::string kSpinAuto = "Spin Auto";

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr double kDriveDeadband = 0.05;
}  // namespace OIConstants

namespace GyroConstants {
    constexpr units::time::second_t kPrintPeriod{1000_ms};
}

namespace ElevatorConstants {
    const int kLeadElevatorMotorCanId = 19;
    const int kFollowerElevatorMotorCanId = 7;

    const int kBottomLimitSwitchPort = 1;

    // Placeholder values
    const double kElevatorP = 280.0;
    const double kElevatorI = 0.0;
    const double kElevatorD = 0.0;
    const double kElevatorIZone = 0.0;
    const double kElevatorFF = 0.0;

    constexpr units::revolutions_per_minute_t kMaxRpm = 5676_rpm;

    // Placeholder values
    constexpr units::meter_t kMinDistance = 0_in;
    constexpr units::meter_t kMaxDistance = 20_in;
    constexpr units::meter_t kRelativeDistancePerRev = 5.51977829236_in / 36; // 36:1 ratio gearbox
    constexpr units::meters_per_second_t kDefaultVelocity = 0.66_mps;
    constexpr double kVelocityScalar = 1.0;
    constexpr units::meter_t kTolerance = 0.5_in;

    // Placeholder
    const subzero::SingleAxisMechanism kElevatorMechanism {
    // min length
    2_in,
    // angle
    90_deg,
    // line width
    10.0,
    // color
    subzero::ColorConstants::kRed};

    const frc::TrapezoidProfile<units::meter>::Constraints kElevatorProfileConstraints{1_fps * 10, 0.75_fps_sq * 20};
};

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

namespace ClimberConstants{
    constexpr int kArmMotorId = 14;
    
    // Placeholder values
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
    
    static const subzero::SingleAxisMechanism kClimberMechanism = {
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