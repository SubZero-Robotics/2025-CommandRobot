#pragma once

#include <frc/RobotBase.h>

#include <string>

#include <subzero/singleaxis/BaseSingleAxisSubsystem.h>
#include <subzero/motor/PidMotorController.h>
#include <subzero/motor/SimPidMotorController.h>
#include <subzero/singleaxis/LinearSingleAxisSubsystem.h>

#include "Constants.h"

class ElevatorSubsystem : public subzero::LinearSingleAxisSubsystem<subzero::IPidMotorController> {
public:
    explicit ElevatorSubsystem() 
    : subzero::LinearSingleAxisSubsystem<subzero::IPidMotorController>{
        "Elevator Subsystem",
         dynamic_cast<subzero::IPidMotorController&>(m_elevatorController),
        {    // Min distance
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
             // Conversion function to output values in different
             // units in shuffleboard
             std::nullopt,
             // Ignore soft limits if `true` is returned
             []() { return false; },
             // Trapazoid profile constraints
             ElevatorConstants::kElevatorProfileConstraints
        },
            nullptr        
    } {}

private:
    rev::spark::SparkMax m_motor{
        ElevatorConstants::kElevatorMotorCanId,
        rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController m_pidController = m_motor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder m_enc = m_motor.GetEncoder();
    rev::spark::SparkAbsoluteEncoder m_absEnc = m_motor.GetAbsoluteEncoder();
    subzero::PidSettings m_elevatorPidSettings = {
      ElevatorConstants::kElevatorP, ElevatorConstants::kElevatorI, ElevatorConstants::kElevatorD,
      ElevatorConstants::kElevatorIZone, ElevatorConstants::kElevatorFF, true};
    SparkMaxPidController m_elevatorController{"Elevator",
                                   m_motor,
                                   m_enc,
                                   m_pidController,
                                   m_elevatorPidSettings,
                                   &m_absEnc,
                                   ElevatorConstants::kMaxRpm};

//   subzero::SimPidMotorController simElevatorController{"Sim Elevator", elevatorPidSettings,
//                                                   ElevatorConstants::kMaxRpm};
};