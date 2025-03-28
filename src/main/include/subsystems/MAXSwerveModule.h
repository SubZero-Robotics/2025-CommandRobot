// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkFlex.h>
#include <rev/SparkRelativeEncoder.h>

using namespace rev::spark;

class MAXSwerveModule {
 public:
  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  MAXSwerveModule(int driveCANId, int turningCANId,
                  double chassisAngularOffset);

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  frc::SwerveModuleState GetState() const;

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  frc::SwerveModulePosition GetPosition() const;

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  void SetDesiredState(const frc::SwerveModuleState& state);

  frc::SwerveModuleState GetDesiredState() { return m_desiredState; }

  /**
   * Zeroes all the SwerveModule encoders.
   */
  void ResetEncoders();

  frc::SwerveModuleState GetSimState() const;

  frc::SwerveModulePosition GetSimPosition() const;

  frc::Rotation2d GetRotation() const;

 private:
  void simUpdateDrivePosition(
    const frc::SwerveModuleState& desiredState);

  SparkFlex m_drivingSpark;
  SparkMax m_turningSpark;

  SparkRelativeEncoder m_drivingEncoder = m_drivingSpark.GetEncoder();
  SparkAbsoluteEncoder m_turningAbsoluteEncoder =
      m_turningSpark.GetAbsoluteEncoder();

  SparkClosedLoopController m_drivingClosedLoopController =
      m_drivingSpark.GetClosedLoopController();
  SparkClosedLoopController m_turningClosedLoopController =
      m_turningSpark.GetClosedLoopController();

  double m_chassisAngularOffset = 0;
  frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0},
                                        frc::Rotation2d()};

  units::meter_t m_simDriveEncoderPosition;
  units::meters_per_second_t m_simDriveEncoderVelocity;

  units::radian_t m_simCurrentAngle;
};