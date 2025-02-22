#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>
#include <rev/SparkLowLevel.h>
#include <rev/config/SparkMaxConfig.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem(const int id, double hasGamePieceCurrent, bool idleOnBrakeMode) 
    : m_motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless}, m_hasGamePieceCurrent{hasGamePieceCurrent} {
        m_config.SetIdleMode(idleOnBrakeMode ? rev::spark::SparkBaseConfig::IdleMode::kBrake 
            : rev::spark::SparkBaseConfig::IdleMode::kCoast);
        
        m_motor.Configure(m_config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                      rev::spark::SparkBase::PersistMode::kPersistParameters);
    }

    frc2::CommandPtr MoveAtPercent(double percent);

    frc2::CommandPtr Stop();

    bool HasGamePiece();
private:
    rev::spark::SparkMax m_motor;
    rev::spark::SparkMaxConfig m_config;

    double m_hasGamePieceCurrent;
};