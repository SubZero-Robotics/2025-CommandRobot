#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>
#include <rev/SparkLowLevel.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem(const int id, double hasGamePieceCurrent) 
    : m_motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless}, m_hasGamePieceCurrent{hasGamePieceCurrent} {}

    frc2::CommandPtr MoveAtPercent(double percent);

    frc2::CommandPtr Stop();

    bool HasGamePiece();
private:
    rev::spark::SparkMax m_motor;

    double m_hasGamePieceCurrent;
};