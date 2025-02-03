#pragma once

#include <frc/RobotBase.h>
#include <subzero/motor/IPidMotorController.h>
#include <subzero/motor/PidMotorController.h>
#include <subzero/motor/SimPidMotorController.h>
#include <subzero/singleaxis/RotationalSingleAxisSubsystem.h>
#include "constants/ArmConstants.h"
#include "Constants.h"

class AlgaeArmSubsystem : public subzero::RotationalSingleAxisSubsystem<subzero::IPidMotorController> {
 public:
  explicit AlgaeArmSubsystem(frc::MechanismObject2d* node = nullptr)
      : subzero::RotationalSingleAxisSubsystem<subzero::IPidMotorController>{
            "Arm",
            frc::RobotBase::IsReal()
                ? dynamic_cast<subzero::IPidMotorController&>(armController)
                : dynamic_cast<subzero::IPidMotorController&>(simArmController),
            {
             AlgaeArmConstants::kHomeRotation,
             // Max distance
             AlgaeArmConstants::kMaxRotation,
             // Distance per revolution of relative encoder
             AlgaeArmConstants::kArmRelativeDistancePerRev,
             // Distance per revolution of absolute encoder
             AlgaeArmConstants::kArmAbsoluteDistancePerRev,
             // Default velocity
             AlgaeArmConstants::kDefaultVelocity,
             // Velocity scalar
             AlgaeArmConstants::kVelocityScalar,
             // Tolerance
             AlgaeArmConstants::kTolerance,
             // Min limit switch
             std::nullopt,
             // Max limit switch
             std::nullopt,
             // Reversed
             true,
             // Mechanism2d
             AlgaeArmConstants::kArmMechanism,
             // Conversion Function
             std::nullopt,

             [] { return false; }, AlgaeArmConstants::kRotationalAxisConstraints},
            AlgaeArmConstants::kArmLength,
            nullptr} {
    //m_Motor.SetIdleMode(rev::spark::SparkBase::IdleMode::kBrake);



            }

  

 private:
  rev::spark::SparkMax m_Motor{AlgaeArmConstants::kArmMotorId,
                               rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkClosedLoopController m_PidController = m_Motor.GetClosedLoopController();
  rev::spark::SparkRelativeEncoder m_enc = m_Motor.GetEncoder();
  rev::spark::SparkAbsoluteEncoder m_absEnc = m_Motor.GetAbsoluteEncoder();
  subzero::PidSettings armPidSettings = {
      AlgaeArmConstants::kArmP, AlgaeArmConstants::kArmI, AlgaeArmConstants::kArmD,
      AlgaeArmConstants::kArmIZone, AlgaeArmConstants::kArmFF};
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