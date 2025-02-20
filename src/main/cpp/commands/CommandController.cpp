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

frc2::CommandPtr CommandController::SetElevatorZeroPosition() {
    return m_subsystems.coralArm->MoveToPositionAbsolute(CoralArmConstants::kMinRotation)
    .AndThen(frc2::InstantCommand(
        [this]() {
            m_subsystems.elevator->SetEncoderPosition(CommandConstants::kElevatorStartPosition);
        }
    ).ToPtr())
    .AndThen(m_subsystems.elevator->MoveToPositionAbsolute(ElevatorConstants::kMinDistance));
}