#include <subsystems/IntakeSubsystem.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/FunctionalCommand.h>

frc2::CommandPtr IntakeSubsystem::MoveAtPercent(double percent) {
    return frc2::FunctionalCommand(
    // On init
    []() {},
    // On execute
    [this, percent]() {
        m_motor.Set(percent);
    },
    // On end
    [this](bool interupted) {
        m_motor.StopMotor();
    },
    // Is finished
    []() {
        return false;
    },
    // Requirements
    {this}
    ).ToPtr();
}

frc2::CommandPtr IntakeSubsystem::Stop() {
    return frc2::InstantCommand(
        [this]() {
            m_motor.StopMotor();
        },
        {this}
    ).ToPtr();
}