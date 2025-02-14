
#include "subsystems/AlgaeArmSubsystem.h"

#include <frc2/command/RunCommand.h>

IntakeSubsystem& AlgaeArmSubsystem::GetIntakeSubsystem() {
    return m_intake;
}