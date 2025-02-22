#include "commands/CommandController.h"

#include <frc2/command/FunctionalCommand.h>

#include "Constants.h"

frc2::CommandPtr CommandController::MoveToPositionL1() {
    return m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorL1Position)
    .RaceWith(m_subsystems.coralIntake->MoveAtPercent(CommandConstants::kCoralIntakeRetainCoralSpeed))
    .AndThen(m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralL1Position))
    .AndThen(m_subsystems.algaeArm->MoveToPositionAbsolute(CommandConstants::kAlgaeArmReefPosition));
}

frc2::CommandPtr CommandController::MoveToPositionL2() {
    return m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorL2Position)
    .RaceWith(m_subsystems.coralIntake->MoveAtPercent(CommandConstants::kCoralIntakeRetainCoralSpeed))
    .AndThen(m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralL2Position))
    .AndThen(m_subsystems.algaeArm->MoveToPositionAbsolute(CommandConstants::kAlgaeArmReefPosition));
}

frc2::CommandPtr CommandController::MoveToPositionL3() {
    return m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorL3Position)
    .RaceWith(m_subsystems.coralIntake->MoveAtPercent(CommandConstants::kCoralIntakeRetainCoralSpeed))
    .AndThen(m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralL3Position))
    .AndThen(m_subsystems.algaeArm->MoveToPositionAbsolute(CommandConstants::kAlgaeArmReefPosition));
}

frc2::CommandPtr CommandController::FeedCoral() {
    return m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralFeedPosition)
    .AndThen(m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorFeedPosition))
    .RaceWith(m_subsystems.coralIntake->MoveAtPercent(CommandConstants::kCoralIntakeRetainCoralSpeed))
    .AndThen(m_subsystems.coralIntake->MoveAtPercent(CommandConstants::kCoralFeedSpeed)
    .Until([this]() {
        return m_subsystems.coralIntake->HasGamePiece();
    })
    .WithTimeout(CommandConstants::kCoralFeedTimeout));
}

frc2::CommandPtr CommandController::ExpelCoral() {
    return m_subsystems.coralIntake->MoveAtPercent(CommandConstants::kCoralExpelSpeed);
}

frc2::CommandPtr CommandController::IntakeAlgae() {
    return m_subsystems.algaeArm->MoveToPositionAbsolute(CommandConstants::kAlgaeIntakePosition)
    .AndThen(m_subsystems.algaeIntake->MoveAtPercent(CommandConstants::kAlgaeIntakeSpeed))
    .Until([this]() {
        return m_subsystems.algaeIntake->HasGamePiece();
    })
    .WithTimeout(CommandConstants::kAlgaeIntakeTimeout);
}

frc2::CommandPtr CommandController::ExpelAlgae() {
    return m_subsystems.algaeIntake->MoveAtPercent(CommandConstants::kAlgaeExpelSpeed);
}

frc2::CommandPtr CommandController::ClimberDown() {
    return m_subsystems.climber->MoveToPositionAbsolute(CommandConstants::kClimberDownAngle);
}

frc2::CommandPtr CommandController::ClimberUp() {
    return m_subsystems.climber->MoveToPositionAbsolute(CommandConstants::kClimberUpAngle);
}

frc2::CommandPtr CommandController::RemoveAlgaeFromL2() {
    return m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorRemoveAlgaeFromL2Position)
    .AndThen(m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralArmRemoveAlgaeFromL2Position))
    .AndThen(m_subsystems.coralIntake->MoveAtPercent(CommandConstants::kCoralFeedSpeed))
    .WithTimeout(CommandConstants::kRemoveAlgaeFromReefTimeout);
    ;
}

frc2::CommandPtr CommandController::RemoveAlgaeFromL3() {
    return m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorRemoveAlgaeFromL3Position)
    .AndThen(m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralArmRemoveAlgaeFromL3Position))
    .AndThen(m_subsystems.coralIntake->MoveAtPercent(CommandConstants::kCoralFeedSpeed))
    .WithTimeout(CommandConstants::kRemoveAlgaeFromReefTimeout);
}
