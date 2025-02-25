
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
#include <frc2/command/DeferredCommand.h>
#include <frc2/command/StartEndCommand.h>

#include <iostream>
#include <memory>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "subsystems/AlgaeArmSubsystem.h"
#include "subsystems/CoralArmSubsystem.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  m_chooser.SetDefaultOption(AutoConstants::kDefaultAutoName, AutoConstants::kDefaultAutoName);

  frc::SmartDashboard::PutData(&m_chooser);
  m_chooser.SetDefaultOption(AutoConstants::kCenterToCenterAuto, AutoConstants::kCenterToCenterAuto);
  m_chooser.AddOption(AutoConstants::kFarLeftAuto, AutoConstants::kFarLeftAuto);
  m_chooser.AddOption(AutoConstants::kFarRightAuto, AutoConstants::kFarRightAuto);
      
  // pathplanner::NamedCommands::registerCommand("Test Command", frc2::cmd::Print("Test Command"));
  pathplanner::NamedCommands::registerCommand("To L1 Position", std::move(m_commandController.MoveToPositionL1()));
  pathplanner::NamedCommands::registerCommand("To L2 Position", std::move(m_commandController.MoveToPositionL2()));
  pathplanner::NamedCommands::registerCommand("To L3 Position", std::move(m_commandController.MoveToPositionL3()));
  pathplanner::NamedCommands::registerCommand("To Feed Position", std::move(m_commandController.FeedCoral()));
  pathplanner::NamedCommands::registerCommand("Remove Algae", std::move(m_commandController.RemoveAlgaeFromL3()));
  pathplanner::NamedCommands::registerCommand("Home Elevator", std::move(m_commandController.HomeElevator()));
  pathplanner::NamedCommands::registerCommand("To Feed Position", std::move(m_commandController.FeedCoral()));
  pathplanner::NamedCommands::registerCommand("Expel Coral", std::move(m_commandController.ExpelCoral().WithTimeout(CommandConstants::kExpelCoralTimeout)));

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

    // m_climber.SetDefaultCommand(
    //   frc2::RunCommand(
    //     [this]() {
    //       m_climber.RunMotorPercentage((-m_driverController.GetLeftTriggerAxis() 
    //         + m_driverController.GetRightTriggerAxis()) * ClimberConstants::kPercentageScalar);

    //         std::cout << "Controller input: " << (-m_driverController.GetLeftTriggerAxis() 
    //         + m_driverController.GetRightTriggerAxis()) * ClimberConstants::kPercentageScalar
    //         << ", Current relative position: " << m_climber.GetCurrentPosition().value() << std::endl;
    //     },
    //     {&m_climber}
    //   )
    // );

  // Has to be raised before match so coral arm is within frame perimeter
  // m_elevator.SetEncoderPosition(ElevatorConstants::kElevatorStartPosition);
}

void RobotContainer::ConfigureBindings() {
  // NOT FINAL

  m_operatorController.POVLeft().OnTrue(m_commandController.MoveToPositionL1());
  m_operatorController.POVUp().OnTrue(m_commandController.MoveToPositionL2());
  m_operatorController.POVRight().OnTrue(m_commandController.MoveToPositionL3());
  m_operatorController.POVDown().OnTrue(m_commandController.FeedCoral());

  m_operatorController.A().WhileTrue(m_commandController.ExpelCoral());
  m_operatorController.B().WhileTrue(m_coralIntake.MoveAtPercent(CommandConstants::kCoralFeedSpeed));
  m_operatorController.X().WhileTrue(m_commandController.ExpelAlgae());
  m_operatorController.Y().WhileTrue(m_commandController.IntakeAlgae());

  m_operatorController.LeftBumper().OnTrue(m_commandController.RemoveAlgaeFromL2());
  m_operatorController.RightBumper().OnTrue(m_commandController.RemoveAlgaeFromL3());

  m_driverController.LeftBumper().OnTrue(m_commandController.HomeElevator());
  m_driverController.RightBumper().OnTrue(frc2::InstantCommand(
    [this]() {
      std::cout << "Resetting rotation" << std::endl;
      m_drive.ResetRotation();
    }
  ).ToPtr());

  // m_operatorController.LeftBumper().WhileTrue(m_commandController.ExpelAlgae());
  // m_operatorController.RightBumper().WhileTrue(m_commandController.ExpelCoral());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  m_autoSelected = m_chooser.GetSelected();

  std::cout << "Auto Selected: \"" << m_autoSelected << "\".\n";

  auto command = pathplanner::PathPlannerAuto(m_autoSelected);
  auto rot = command.getStartingPose().Rotation().Degrees();

  auto offset = m_drive.GetHeading() - rot;
  
  return pathplanner::PathPlannerAuto(m_autoSelected)
    .AndThen(frc2::InstantCommand([this, offset]() { m_drive.OffsetRotation(offset); }).ToPtr());
}

void RobotContainer::Periodic() {
  frc::SmartDashboard::PutData("Robot Elevator", &m_elevatorMech);

  // frc::SmartDashboard::PutData("Zero Odometry", new frc2::InstantCommand([this]() { m_drive.ResetRotation(); }));
}

void RobotContainer::Initialize() {
  m_elevator.OnInit();
  m_coralArm.OnInit();
  m_algaeArm.OnInit();
  // m_climber.OnInit();
}