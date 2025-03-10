// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADIS16470_IMU.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <networktables/StructArrayTopic.h>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>
#include <subzero/vision/PhotonVisionEstimators.h>
#include <subzero/target/ITurnToTarget.h>
#include <subzero/logging/ConsoleLogger.h>
#include <photon/PhotonCamera.h>
#include "Constants.h"
#include "MAXSwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  explicit DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);

  void Drive(frc::ChassisSpeeds speeds);

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  void SetX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  units::degree_t GetHeading();

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  void OffsetRotation(frc::Rotation2d rot);

  void ResetRotation();

  frc::SwerveDriveKinematics<4> m_driveKinematics{
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2}};

  void AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                            units::second_t timestamp);
  void AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                            units::second_t timestamp,
                            const Eigen::Vector3d& stdDevs);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::ChassisSpeeds getRobotRelativeSpeeds();

  MAXSwerveModule m_frontLeft;
  MAXSwerveModule m_rearLeft;
  MAXSwerveModule m_frontRight;
  MAXSwerveModule m_rearRight;

  // Gyro Sensor
  ctre::phoenix6::hardware::Pigeon2 m_pidgey{DriveConstants::kPigeonCanId, "rio"};
  units::time::second_t currentTime{frc::Timer::GetFPGATimestamp()};

  // Odometry class for tracking robot pose
  // 4 defines the number of modules
  frc::SwerveDriveOdometry<4> m_odometry;

  frc::SwerveDrivePoseEstimator<4> poseEstimator{
      m_driveKinematics,
      GetHeading(),
      {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
       m_frontRight.GetPosition(), m_rearRight.GetPosition()},
      frc::Pose2d{0_m, 0_m, 0_rad},
      VisionConstants::kDrivetrainStd,
      VisionConstants::kVisionStd};
  nt::StructArrayPublisher<frc::SwerveModuleState> m_publisher;
  nt::StructArrayPublisher<frc::SwerveModuleState> m_desiredPublisher;

  // Pose viewing
  frc::Field2d m_field;
  frc::Pose2d m_lastGoodPosition;

  subzero::PhotonVisionEstimators* m_vision;
  photon::PhotonCamera m_camera{"purplePipeCam"};
};


