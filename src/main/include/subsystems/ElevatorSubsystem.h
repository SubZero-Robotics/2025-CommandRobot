#pragma once

#include <frc/RobotBase.h>

#include <string>

#include <subzero/singleaxis/BaseSingleAxisSubsystem.h>
#include <subzero/motor/PidMotorController.h>
#include <subzero/motor/SimPidMotorController.h>
#include <subzero/singleaxis/LinearSingleAxisSubsystem.h>
#include <frc/smartdashboard/Mechanism2d.h>

#include "Constants.h"

class ElevatorSubsystem : public subzero::LinearSingleAxisSubsystem<subzero::IPidMotorController> {
public:
    explicit ElevatorSubsystem(frc::MechanismObject2d* node = nullptr) 
    : subzero::LinearSingleAxisSubsystem<subzero::IPidMotorController>{
        "Elevator Subsystem",
        frc::RobotBase::IsReal() ?
         dynamic_cast<subzero::IPidMotorController&>(m_elevatorController) :
         dynamic_cast<subzero::IPidMotorController&>(simElevatorController),
        {    // Min distance
             ElevatorConstants::kMinDistance,
             // Max distance
             ElevatorConstants::kMaxDistance,
             // Distance per revolution of relative encoder
             ElevatorConstants::kRelativeDistancePerRev,
             // Distance per revolution of absolute encoder
             std::nullopt,
             // Default velocity
             ElevatorConstants::kDefaultVelocity,
             // Velocity scalar
             ElevatorConstants::kVelocityScalar,
             // Tolerance
             ElevatorConstants::kTolerance,
             // Min limit switch
             &m_bottomLimitSwitchPort,
             // Max limit switch
             std::nullopt,
             // Reversed
             false,
             // Mechanism2d
             ElevatorConstants::kElevatorMechanism,
             // Conversion function to output values in different
             // units in shuffleboard
             std::nullopt,
             // Ignore soft limits if `true` is returned
             []() { return false; },
             // Trapazoid profile constraints
             ElevatorConstants::kElevatorProfileConstraints
        },
            node        
    } {
        m_followerConfig.Follow(m_leadMotor, true);
        m_followerMotor.Configure(m_followerConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, 
                                  rev::spark::SparkBase::PersistMode::kPersistParameters);
    }

private:
    rev::spark::SparkMax m_leadMotor{
        ElevatorConstants::kLeadElevatorMotorCanId,
        rev::spark::SparkLowLevel::MotorType::kBrushless};

    rev::spark::SparkMax m_followerMotor{
        ElevatorConstants::kFollowerElevatorMotorCanId,
        rev::spark::SparkLowLevel::MotorType::kBrushless};

    // Here so we can set the foller motor as a follower
    rev::spark::SparkMaxConfig m_followerConfig;

    frc::DigitalInput m_bottomLimitSwitchPort{ElevatorConstants::kBottomLimitSwitchPort};

    rev::spark::SparkClosedLoopController m_pidController = m_leadMotor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder m_enc = m_leadMotor.GetEncoder();
    subzero::PidSettings m_elevatorPidSettings = {
      ElevatorConstants::kElevatorP, ElevatorConstants::kElevatorI, ElevatorConstants::kElevatorD,
      ElevatorConstants::kElevatorIZone, ElevatorConstants::kElevatorFF};
    SparkMaxPidController m_elevatorController{"Elevator",
                                   m_leadMotor,
                                   m_enc,
                                   m_pidController,
                                   m_elevatorPidSettings,
                                   nullptr,
                                   ElevatorConstants::kMaxRpm};

  subzero::SimPidMotorController simElevatorController{"Sim Elevator", m_elevatorPidSettings,
                                                  ElevatorConstants::kMaxRpm};
};