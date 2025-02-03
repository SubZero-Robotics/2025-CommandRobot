// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <iostream>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // m_chooser.SetDefaultOption(AutoConstants::kDefaultAutoName, AutoConstants::kDefaultAutoName);

  frc::SmartDashboard::PutData(&m_chooser);
  m_chooser.SetDefaultOption(AutoConstants::kOutAuto, AutoConstants::kSpinAuto);
  m_chooser.AddOption(AutoConstants::kSpinAuto, AutoConstants::kSpinAuto);

  // Configure the button bindings
  ConfigureBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureBindings() {
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  auto command = pathplanner::PathPlannerAuto("Spin Auto");
  auto rot = command.getStartingPose().Rotation().Degrees();

  auto offset = m_drive.GetHeading() - rot;

  m_autoSelected = m_chooser.GetSelected();
  
  std::cout << "Auto Selected: \"" << m_autoSelected << "\".\n";

  return pathplanner::PathPlannerAuto(m_autoSelected)
    .AndThen(frc2::InstantCommand([this, offset]() { m_drive.OffsetRotation(offset); }).ToPtr());
}
