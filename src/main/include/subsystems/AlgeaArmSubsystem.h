#pragma once

#include <frc/RobotBase.h>
#include <subzero/motor/PidMotorController.h>
#include <subzero/motor/SimPidMotorController.h>
#include <subzero/singleaxis/RotationalSingleAxisSubsystem.h>
#include "constants/ArmConstants.h"
#include "Constants.h"

class ArmSubsystem : public RotationalSingleAxisSubsystem<IPidMotorController> {
 public:
  explicit ArmSubsystem(frc::MechanismObject2d* node = nullptr)
      : RotationalSingleAxisSubsystem<IPidMotorController>{
            "Arm",
            frc::RobotBase::IsReal()
                ? dynamic_cast<IPidMotorController&>(armController)
                : dynamic_cast<IPidMotorController&>(simArmController),
            {
             AlgeaArmConstants::kHomeRotation,
             // Max distance
             AlgeaArmConstants::kMaxRotation,
             // Distance per revolution of relative encoder
             AlgeaArmConstants::kArmRelativeDistancePerRev,
             // Distance per revolution of absolute encoder
             AlgeaArmConstants::kArmAbsoluteDistancePerRev,
             // Default velocity
             AlgeaArmConstants::kDefaultVelocity,
             // Velocity scalar
             AlgeaArmConstants::kVelocityScalar,
             // Tolerance
             AlgeaArmConstants::kTolerance,
             // Min limit switch
             std::nullopt,
             // Max limit switch
             std::nullopt,
             // Reversed
             true,
             // Mechanism2d
             AlgeaArmConstants::kArmMechanism,
             // Conversion Function
             std::nullopt,

             [] { return false; }, AlgeaArmConstants::kRotationalAxisConstraints},
            AlgeaArmConstants::kArmLength,
            node} {
    m_Motor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);



            }

  

 private:
  rev::CANSparkMax m_Motor{AlgeaArmConstants::kArmMotorId,
                               rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkPIDController m_PidController = m_Motor.GetPIDController();
  rev::SparkRelativeEncoder m_enc = m_Motor.GetEncoder();
  rev::SparkAbsoluteEncoder m_absEnc = m_Motor.GetAbsoluteEncoder();
  subzero::PidSettings armPidSettings = {
      AlgaeArmConstants::kArmP, AlgaeArmConstants::kArmI, AlgaeArmConstants::kArmD,
      AlgaeArmConstants::kArmIZone, AlgeaArmConstants::kArmFF};
  SparkMaxController armController{"Arm",
                                   m_Motor,
                                   m_enc,
                                   m_PidController,
                                   armPidSettings,
                                   &m_absEnc,
                                   AlgaeArmConstants::kMaxRpm};
  subzero::SimPidMotorController simArmController{"Sim Arm", armPidSettings,
                                                  AlgaeArmConstants::kMaxRpm};
};