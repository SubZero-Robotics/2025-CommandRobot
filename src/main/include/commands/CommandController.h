#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/AlgaeArmSubsystem.h"
#include "subsystems/CoralArmSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ClimberSubsystem.h"

typedef struct {
    AlgaeArmSubsystem *algaeArm;
    CoralArmSubsystem *coralArm;
    ElevatorSubsystem *elevator;
    IntakeSubsystem *coralIntake;
    IntakeSubsystem *algaeIntake;
    ClimberSubsystem *climber;
} Subsystems_t;

class CommandController {
public:
    CommandController(Subsystems_t subsystems) : m_subsystems{subsystems} {}

    frc2::CommandPtr MoveToPositionL1();
    frc2::CommandPtr MoveToPositionL2();
    frc2::CommandPtr MoveToPositionL3();

    frc2::CommandPtr FeedCoral();
    frc2::CommandPtr ExpelCoral();

    frc2::CommandPtr IntakeAlgae();
    frc2::CommandPtr ExpelAlgae();

    frc2::CommandPtr RemoveAlgaeFromL2();
    frc2::CommandPtr RemoveAlgaeFromL3();

    frc2::CommandPtr HomeElevator();

    frc2::CommandPtr ClimbUp();
    frc2::CommandPtr ClimbDown();
private:
    Subsystems_t m_subsystems;
};