#include "commands/CommandController.h"

#include <frc2/command/FunctionalCommand.h>

#include "Constants.h"

frc2::CommandPtr CommandController::MoveToPositionL1() {
    return m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorL1Position)
    .AndThen(m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralL1Position));
}

frc2::CommandPtr CommandController::MoveToPositionL2() {
    return m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorL2Position)
    .AndThen(m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralL2Position));
}

frc2::CommandPtr CommandController::MoveToPositionL3() {
    return m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorL3Position)
    .AndThen(m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralL3Position));
}

frc2::CommandPtr CommandController::FeedCoral() {
    return m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralFeedPosition)
    .AndThen(m_subsystems.coralIntake->MoveAtPercent(CommandConstants::kCoralIntakeSpeed)
    .Until([this]() {
        return m_subsystems.coralIntake->HasGamePiece();
    })
    .WithTimeout(CoralArmConstants::kIntakeTimeout));
}

frc2::CommandPtr CommandController::ExpelCoral() {
    return frc2::FunctionalCommand(
    // On init
    []() {},
    // On execute
    [this]() {
        m_subsystems.coralIntake->MoveAtPercent(CommandConstants::kCoralExpelSpeed);
    },
    // On end
    [this](bool interupted) {
        m_subsystems.coralIntake->Stop();
    },
    // Is finished
    []() {
        return false;
    },
    // Requirements
    {m_subsystems.coralIntake}
    ).ToPtr();
}

frc2::CommandPtr CommandController::IntakeAlgae() {
    return m_subsystems.algaeArm->MoveToPositionAbsolute(CommandConstants::kAlgaeIntakePosition)
    .AndThen(frc2::FunctionalCommand(
    // On init
    []() {},
    // On execute
    [this]() {
        m_subsystems.algaeIntake->MoveAtPercent(CommandConstants::kAlgaeIntakeSpeed);
    },
    // On end
    [this](bool interupted) {
        m_subsystems.algaeIntake->Stop();
    },
    // Is finished
    []() {
        return false;
    },
    // Requirements
    {m_subsystems.algaeIntake}
    ).ToPtr())
        .Until([this]() {
        return m_subsystems.coralIntake->HasGamePiece();
    })
    .WithTimeout(AlgaeArmConstants::kIntakeTimeout);
}

frc2::CommandPtr CommandController::ExpelAlgae() {
    return frc2::FunctionalCommand(
    // On init
    []() {},
    // On execute
    [this]() {
        m_subsystems.algaeIntake->MoveAtPercent(CommandConstants::kAlgaeExpelSpeed);
    },
    // On end
    [this](bool interupted) {
        m_subsystems.algaeIntake->Stop();
    },
    // Is finished
    []() {
        return false;
    },
    // Requirements
    {m_subsystems.algaeIntake}
    ).ToPtr();
}

frc2::CommandPtr CommandController::ClimberDown() {
    return m_subsystems.climber->MoveToPositionAbsolute(CommandConstants::kClimberDownAngle);
}

frc2::CommandPtr CommandController::ClimberUp() {
    return m_subsystems.climber->MoveToPositionAbsolute(CommandConstants::kClimberUpAngle);
}