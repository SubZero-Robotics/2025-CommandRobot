// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <hal/FRCUsageReporting.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
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
      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::radian_t{
                     pidgey.GetYaw().GetValue()}),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {
  // Usage reporting for MAXSwerve template
  HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
             HALUsageReporting::kRobotDriveSwerve_MaxSwerve);

    /* Configure Pigeon2 */

  pidgey.SetYaw(144_deg, 100_ms); // Set our yaw to 144 degrees and wait up to 100 milliseconds for the setter to take affect
  pidgey.GetYaw().WaitForUpdate(100_ms); // And wait up to 100 milliseconds for the yaw to take affect
  std::cout << "Set the yaw to 144 degrees, we are currently at " << pidgey.GetYaw() << std::endl;
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(frc::Rotation2d(units::radian_t{
                        pidgey.GetYaw().GetValue()}),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
    auto &yaw = pidgey.GetYaw();
  if (frc::Timer::GetFPGATimestamp() - currentTime >= GyroConstants::kPrintPeriod) {
  //   currentTime += GyroConstants::kPrintPeriod;
    std::cout << yaw.GetValue().value() << std::endl;
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
  //   auto &gravityZ = pidgey.GetGravityVectorZ(false);
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
  }
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

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{
                    pidgey.GetYaw().GetValue()}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
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
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
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
             units::radian_t{pidgey.GetYaw().GetValue()})
      .Degrees();
}

void DriveSubsystem::ZeroHeading() { pidgey.Reset(); }

double DriveSubsystem::GetTurnRate() {
  return -pidgey.GetAngularVelocityZWorld().GetValue().value();
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
        m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}