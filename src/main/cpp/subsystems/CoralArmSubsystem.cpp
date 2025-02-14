#include "subsystems/CoralArmSubsystem.h"

#include <frc2/command/RunCommand.h>

frc2::CommandPtr CoralArmSubsystem::MoveAtSpeed(double speed) {
    return frc2::FunctionalCommand(
    // On init
    []() {},
    // On execute
    [this, speed]() {
        m_intakeMotor.Set(speed);
        std::cout << "Running motor";
    },
    // On end
    [this](bool interupted) {
        m_intakeMotor.StopMotor();
    },
    // Is finished
    []() {
        return false;
    },
    // Requirements
    {this}
    ).ToPtr();
}
