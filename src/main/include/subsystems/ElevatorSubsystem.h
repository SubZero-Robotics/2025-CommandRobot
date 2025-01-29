#pragma once

#include <frc/RobotBase.h>
#include <subzero/motor/PidMotorController.h>

#include "Constants.h"

class ElevatorSubsystem : LinearSingleAxisSubsystem<PidMotorController> {
public:
    explicit ElevatorSubsystem() : LinearSingleAxisSubsystem{
        "Elevator Subsystem",
        frc::RobotBase::IsReal()
                ? dynamic_cast<IPidMotorController&>(armController)
                : dynamic_cast<IPidMotorController&>(simArmController),

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
             std::nullopt,

             [] {return false},

             
        }
    } {}

private:
    rev::CANSparkMax m_motor{
        ElevatorConstants::kElevatorMotorCanId,
        rev::CANSparkLowLevel::MotorType::kBrushless};

    rev::SparkPIDController m_pidController = m_motor.GetPIDController();
    rev::SparkRelativeEncoder m_enc = m_motor.GetEncoder();
    rev::SparkAbsoluteEncoder m_absEnc = m_motor.GetAbsoluteEncoder();
    subzero::PidSettings elevatorPidSettings = {
      ElevatorConstants::kElevatorP, ElevatorConstants::kElevatorI, ElevatorConstants::kElevatorD,
      ElevatorConstants::kElevatorIZone, ArmConstants::kArmFF};
    SparkMaxController elevatorController{"Elevator",
                                   m_motor,
                                   m_enc,
                                   m_PidController,
                                   elevatorPidSettings,
                                   &m_absEnc,
                                   ElevatorConstants::kMaxRpm};
  subzero::SimPidMotorController simArmController{"Sim Elevator", elevatorPidSettings,
                                                  IntakingConstants::kMaxRpm};
};