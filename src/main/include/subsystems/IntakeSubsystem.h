#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem(const int id) : m_motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless} {}

    frc2::CommandPtr MoveAtPercent(double percent);

    frc2::CommandPtr Stop();
private:
    rev::spark::SparkMax m_motor;
};