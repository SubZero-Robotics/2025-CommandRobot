#include <subsystems/ElevatorSubsystem.h>

void ElevatorSubsystem::SetEncoderPosition(units::meter_t pos) {
    // Basesingleaxis applies a conversion factor so we don't provide rotations
    m_enc.SetPosition(pos.value());
}

bool ElevatorSubsystem::GetMinLimitSwitch() {
    return m_leadMotor.GetReverseLimitSwitch().Get();
}

bool ElevatorSubsystem::GetMaxLimitSwitch() {
    return m_leadMotor.GetForwardLimitSwitch().Get();
}

void ElevatorSubsystem::Periodic() {
  if (GetMaxLimitSwitch()) {
    std::cout << "Elevator at max" << std::endl;
    SetEncoderPosition(ElevatorConstants::kMaxDistance);
  } else if (GetMinLimitSwitch()) {
    std::cout << "Elevator at min" << std::endl;
    SetEncoderPosition(ElevatorConstants::kMinDistance);
  }

  frc::SmartDashboard::PutBoolean("Forward limit switch state", GetMaxLimitSwitch());
  frc::SmartDashboard::PutBoolean("Reverse limit switch state", GetMinLimitSwitch());

  subzero::LinearSingleAxisSubsystem<subzero::IPidMotorController>::Periodic();
}

void ElevatorSubsystem::UpdatePidSettings(subzero::PidSettings settings) {
    m_elevatorController.UpdatePidSettings(settings);
}