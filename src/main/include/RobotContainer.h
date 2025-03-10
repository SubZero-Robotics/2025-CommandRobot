// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "CommonCompile.h"

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/AlgaeArmSubsystem.h"
#include "subsystems/CoralArmSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "commands/CommandController.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  void Periodic();

  void Initialize();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{
      OperatorConstants::kDriverControllerPort};

    frc2::CommandXboxController m_operatorController{
      OperatorConstants::kOperatorControllerPort};

  // The robot's subsystems are defined here...
   DriveSubsystem m_drive;

  ExampleSubsystem m_subsystem;

  void ConfigureBindings();

  frc::SendableChooser<std::string> m_chooser;
  std::string m_autoSelected;

  frc::Mechanism2d m_elevatorMech{0.5, 0.5};
  frc::MechanismRoot2d* m_root = m_elevatorMech.GetRoot("Elevator", 0.25, 0.25);
  
  ElevatorSubsystem m_elevator{(frc::MechanismObject2d*)m_elevatorMech.GetRoot("Elevator", 0.25, 0.25)};

  AlgaeArmSubsystem m_algaeArm;
  IntakeSubsystem m_algaeIntake{AlgaeArmConstants::kIntakeMotorId, AlgaeArmConstants::kHasAlgaeCurrent, true};

  CoralArmSubsystem m_coralArm;
  IntakeSubsystem m_coralIntake{CoralArmConstants::kIntakeMotorId, CoralArmConstants::kHasCoralCurrent, true};

  Subsystems_t subsystems = {
    .algaeArm = &m_algaeArm,
    .coralArm = &m_coralArm,
    .elevator = &m_elevator,
    .coralIntake = &m_coralIntake,
    .algaeIntake = &m_algaeIntake,
    .climber = NULL
  };

  CommandController m_commandController{subsystems};

  units::second_t m_timeSinceControllerInput;
  units::second_t m_defaultLastCalled;
};