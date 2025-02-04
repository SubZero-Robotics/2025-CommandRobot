#pragma once

#include <frc/RobotBase.h>

#include <string>

#include <subzero/singleaxis/BaseSingleAxisSubsystem.h>
#include <subzero/motor/PidMotorController.h>
#include <subzero/motor/SimPidMotorController.h>
#include <subzero/singleaxis/LinearSingleAxisSubsystem.h>

#include "Constants.h"

class ElevatorSubsystem : subzero::LinearSingleAxisSubsystem<subzero::IPidMotorController> {
public:
    explicit ElevatorSubsystem() 
    : subzero::LinearSingleAxisSubsystem<subzero::IPidMotorController>{
        "Elevator Subsystem",
        elevatorController,
        {   // Min distance
             ElevatorConstants::kMinDistance,
             // Max distance
             ElevatorConstants::kMaxDistance,
             // Distance per revolution of relative encoder
             ElevatorConstants::kRelativeDistancePerRev,
             // Distance per revolution of absolute encoder
             ElevatorConstants::kAbsoluteDistancePerRev,
             // Default velocity
             ElevatorConstants::kDefaultVelocity,
             // Velocity scalar
             ElevatorConstants::kVelocityScalar,
             // Tolerance
             ElevatorConstants::kTolerance,
             // Min limit switch
             std::nullopt,
             // Max limit switch
             std::nullopt,
             // Reversed
             false,
             // Mechanism2d
             ElevatorConstants::kElevatorMechanism,

             std::nullopt,

             []() { return false; },

             ElevatorConstants::kElevatorProfileConstraints
        }        
    } {}

private:
    rev::spark::SparkMax m_motor{
        ElevatorConstants::kElevatorMotorCanId,
        rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController m_pidController = m_motor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder m_enc = m_motor.GetEncoder();
    rev::spark::SparkAbsoluteEncoder m_absEnc = m_motor.GetAbsoluteEncoder();
    subzero::PidSettings elevatorPidSettings = {
      ElevatorConstants::kElevatorP, ElevatorConstants::kElevatorI, ElevatorConstants::kElevatorD,
      ElevatorConstants::kElevatorIZone, ElevatorConstants::kElevatorFF, true};
    SparkMaxPidController elevatorController{"Elevator",
                                   m_motor,
                                   m_enc,
                                   m_pidController,
                                   elevatorPidSettings,
                                   &m_absEnc,
                                   ElevatorConstants::kMaxRpm};

  subzero::SimPidMotorController simArmController{"Sim Elevator", elevatorPidSettings,
                                                  ElevatorConstants::kMaxRpm};
};