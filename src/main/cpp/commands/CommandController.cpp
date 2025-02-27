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
    .AndThen(m_subsystems.algaeArm->MoveToPositionAbsolute(CommandConstants::kAlgaeArmReefPosition))
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
    // .Until([this]() {
    //     return m_subsystems.algaeIntake->HasGamePiece();
    // })
    // .WithTimeout(CommandConstants::kAlgaeIntakeTimeout);
    ;
}

frc2::CommandPtr CommandController::ExpelAlgae() {
    return m_subsystems.algaeIntake->MoveAtPercent(CommandConstants::kAlgaeExpelSpeed);
}

frc2::CommandPtr CommandController::RemoveAlgaeFromL2() {
    return m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorRemoveAlgaeFromL2Position)
    .AndThen(m_subsystems.algaeArm->MoveToPositionAbsolute(CommandConstants::kRemoveAlgaeAlgaeArmPosition))
    .AndThen(m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralArmRemoveAlgaeFromL2Position))
    .AndThen(m_subsystems.coralIntake->MoveAtPercent(CommandConstants::kCoralExpelSpeed))
    .WithTimeout(CommandConstants::kRemoveAlgaeFromReefTimeout)
    .RaceWith(m_subsystems.algaeIntake->MoveAtPercent(CommandConstants::kAlgaeIntakeSpeed))
    ;
}

frc2::CommandPtr CommandController::RemoveAlgaeFromL3() {
    return m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorRemoveAlgaeFromL3Position)
    .AndThen(m_subsystems.algaeArm->MoveToPositionAbsolute(CommandConstants::kRemoveAlgaeAlgaeArmPosition))
    .AndThen(m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralArmRemoveAlgaeFromL3Position))
    .AndThen(m_subsystems.coralIntake->MoveAtPercent(CommandConstants::kCoralFeedSpeed))
    .WithTimeout(CommandConstants::kRemoveAlgaeFromReefTimeout);
}

frc2::CommandPtr CommandController::HomeElevator() {
    return
    m_subsystems.coralArm->MoveToPositionAbsolute(CommandConstants::kCoralFeedPosition)
    .AndThen(frc2::InstantCommand(
        [this]() {
            m_subsystems.elevator->SetEncoderPosition(ElevatorConstants::kMaxDistance);
        }
    ).ToPtr())
    .AndThen(frc2::InstantCommand(
        [this]() {
            std::cout << "Setting homing PID settings" << std::endl;
            m_subsystems.elevator->UpdatePidSettings(ElevatorConstants::kHomePidSettings);
        }
    ).ToPtr())
    .AndThen(m_subsystems.elevator->MoveToPositionAbsolute(ElevatorConstants::kMinDistance))
    .FinallyDo(
        [this]() {
            std::cout << "Setting typical PID settings" << std::endl;
            m_subsystems.elevator->UpdatePidSettings(ElevatorConstants::kElevatorPidSettings);
        }
    );
}

frc2::CommandPtr CommandController::ClimbUp() {
    return m_subsystems.algaeArm->MoveToPositionAbsolute(CommandConstants::kClimbAlgaeArmPosition)
    .AndThen(m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorClimbUpPosition));
}

frc2::CommandPtr CommandController::ClimbDown() {
    return m_subsystems.elevator->MoveToPositionAbsolute(CommandConstants::kElevatorClimbDownPosition);
}