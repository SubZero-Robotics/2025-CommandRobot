#include <subsystems/ElevatorSubsystem.h>

void ElevatorSubsystem::SetEncoderPosition(units::meter_t pos) {
    // Basesingleaxis applies a conversion factor so we don't provide rotations
    m_enc.SetPosition(pos.value());
}