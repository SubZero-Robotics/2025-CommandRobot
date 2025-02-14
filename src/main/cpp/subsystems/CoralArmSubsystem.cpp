#include "subsystems/CoralArmSubsystem.h"

#include <frc2/command/RunCommand.h>

IntakeSubsystem& CoralArmSubsystem::GetIntakeSubsystem() {
    return m_intake;
}