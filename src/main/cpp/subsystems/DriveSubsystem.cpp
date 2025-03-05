// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h" 
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <subzero/drivetrain/SwerveUtils.h>
#include <frc/geometry/Rotation2d.h>
#include <hal/FRCUsageReporting.h>
#include <frc/DriverStation.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/length.h>
#include <iostream>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_odometry{m_driveKinematics,
                 frc::Rotation2d(units::radian_t{
                     m_pidgey.GetYaw().GetValue()}),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {
  // Usage reporting for MAXSwerve template
  HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
             HALUsageReporting::kRobotDriveSwerve_MaxSwerve);

  frc::SmartDashboard::PutData("Field", &m_field);

  pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

   pathplanner::AutoBuilder::configure(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ Drive(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<pathplanner::PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            pathplanner::PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    /* Configure Pigeon2 */

  m_pidgey.SetYaw(0_deg, 20_ms); // Set our yaw to 0 degrees and wait up to 100 milliseconds for the setter to take affect
  m_pidgey.GetYaw().WaitForUpdate(20_ms); // And wait up to 100 milliseconds for the yaw to take affect
  // std::cout << "Set the yaw to 144 degrees, we are currently at " << m_pidgey.GetYaw() << std::endl;
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.

  if (frc::RobotBase::IsReal()) {
    poseEstimator.UpdateWithTime(frc::Timer::GetFPGATimestamp(), GetHeading(),
                                 GetModulePositions());

    auto updatedPose = poseEstimator.GetEstimatedPosition();
    // https://github.com/Hemlock5712/2023-Robot/blob/dd5ac64587a3839492cfdb0a28d21677d465584a/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java#L149
    m_lastGoodPosition = updatedPose;
    m_field.SetRobotPose(updatedPose);

    m_odometry.Update(frc::Rotation2d(units::radian_t{
                        m_pidgey.GetYaw().GetValue()}),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
    auto &yaw = m_pidgey.GetYaw();
  }

  // if (frc::Timer::GetFPGATimestamp() - currentTime >= GyroConstants::kPrintPeriod) {

  //   std::cout << "Yaw: " << yaw.GetValue().value() << std::endl;
  
  //   currentTime += GyroConstants::kPrintPeriod;
  //   /**
  //    * GetYaw automatically calls Refresh(), no need to manually refresh.
  //    *
  //    * StatusSignalValues also have the "ostream <<" operator implemented, to provide
  //    * a useful print of the signal
  //    */
  //   std::cout << "Yaw is " << yaw << " with " << yaw.GetTimestamp().GetLatency().value() << " seconds of latency" << std::endl;

  //   /**
  //    * Get the Gravity Vector Z component StatusSignalValue without refreshing
  //    */
  //   auto &gravityZ = m_pidgey.GetGravityVectorZ(false);
  //   /* This time wait for the signal to reduce latency */
  //   gravityZ.WaitForUpdate(GyroConstants::kPrintPeriod); // Wait up to our period
  //   /**
  //    * This uses the explicit GetValue and GetUnits methods to print, even though it's not
  //    * necessary for the ostream print
  //    */
  //   std::cout << "Gravity Vector in the Z direction is " <<
  //                 gravityZ.GetValue().value() << " " <<
  //                 gravityZ.GetUnits() << " with " <<
  //                 gravityZ.GetTimestamp().GetLatency().value() << " seconds of latency" <<
  //                 std::endl;
  //   /**
  //    * Notice when running this example that the second print's latency is always shorter than the first print's latency.
  //    * This is because we explicitly wait for the signal using the WaitForUpdate() method instead of using the Refresh()
  //    * method, which only gets the last cached value (similar to how Phoenix v5 works).
  //    * This can be used to make sure we synchronously update our control loop from the CAN bus, reducing any latency or jitter in
  //    * CAN bus measurements.
  //    * When the device is on a CANivore, the reported latency is very close to the true latency of the sensor, as the CANivore
  //    * timestamps when it receives the frame. This can be further used for latency compensation.
  //    */
  //   std::cout << std::endl;
  // }
}

wpi::array<frc::SwerveModulePosition, 4U> DriveSubsystem::GetModulePositions()
    const {
  return {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
          m_rearLeft.GetPosition(), m_rearRight.GetPosition()};
}

void DriveSubsystem::SimulationPeriodic() {
  frc::ChassisSpeeds chassisSpeeds = m_driveKinematics.ToChassisSpeeds(
      m_frontLeft.GetState(), m_frontRight.GetState(), m_rearLeft.GetState(),
      m_rearRight.GetState());

  m_simPidgey.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  m_simPidgey.SetRawYaw(
      GetHeading() +
      units::degree_t(chassisSpeeds.omega.convert<units::deg_per_s>().value() *
                      DriveConstants::kPeriodicInterval.value()));

  poseEstimator.Update(GetHeading(), GetModulePositions());
  m_field.SetRobotPose(poseEstimator.GetEstimatedPosition());

  std::cout << "In Sim" << std::endl;
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeed.value() * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeed.value() * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      rot.value() * DriveConstants::kMaxAngularSpeed;

  auto states = m_driveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{
                    m_pidgey.GetYaw().GetValue()}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  m_driveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::Drive(frc::ChassisSpeeds speeds) {
  DriveSubsystem::SetModuleStates(
      m_driveKinematics.ToSwerveModuleStates(speeds));
}

frc::ChassisSpeeds DriveSubsystem::getRobotRelativeSpeeds() {
  auto fl = m_frontLeft.GetState();
  auto fr = m_frontRight.GetState();
  auto rl = m_rearLeft.GetState();
  auto rr = m_rearRight.GetState();

  return m_driveKinematics.ToChassisSpeeds(fl, fr, rl, rr);
}

void DriveSubsystem::SetX() {
  m_frontLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  m_frontRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  m_driveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() {
  return frc::Rotation2d(
             units::radian_t{m_pidgey.GetYaw().GetValue()})
      .Degrees();
}

void DriveSubsystem::ZeroHeading() { m_pidgey.Reset(); }

double DriveSubsystem::GetTurnRate() {
  return -m_pidgey.GetAngularVelocityZWorld().GetValue().value();
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
        m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}

void DriveSubsystem::ResetRotation() {
  m_pidgey.Reset();
}

void DriveSubsystem::OffsetRotation(frc::Rotation2d offset) {
  m_pidgey.SetYaw(offset.Degrees() + m_pidgey.GetYaw().GetValue());
    m_odometry.ResetPosition(
      m_pidgey.GetYaw().GetValue(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
        m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      m_odometry.GetPose());
}

void DriveSubsystem::AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                                          units::second_t timestamp) {
  // poseEstimator.AddVisionMeasurement(visionMeasurement, timestamp);
}

void DriveSubsystem::AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                                          units::second_t timestamp,
                                          const Eigen::Vector3d& stdDevs) {
  // wpi::array<double, 3> newStdDevs{stdDevs(0), stdDevs(1), stdDevs(2)};
  // poseEstimator.AddVisionMeasurement(visionMeasurement, timestamp, newStdDevs);
}