#pragma once

#include <frc/RobotBase.h>
#include <frc/smartdashboard/Mechanism2d.h>

#include <subzero/motor/IPidMotorController.h>
#include <subzero/motor/PidMotorController.h>
#include <subzero/motor/SimPidMotorController.h>
#include <subzero/singleaxis/RotationalSingleAxisSubsystem.h>

#include "Constants.h"

class ClimberSubsystem : public subzero::RotationalSingleAxisSubsystem<subzero::IPidMotorController> {
    public:
     explicit ClimberSubsystem(frc::MechanismObject2d* node = nullptr) 
         : subzero::RotationalSingleAxisSubsystem<subzero::IPidMotorController>{
            "Climber Arm",
            frc::RobotBase::IsReal()
                ? dynamic_cast<subzero::IPidMotorController&>(climberController)
                : dynamic_cast<subzero::IPidMotorController&>(simClimberController),
            {
             ClimberConstants::kHomeRotation,
             // Max distance
             ClimberConstants::kMaxRotation,
             // Distance per revolution of relative encoder
             ClimberConstants::kRelativeDistancePerRev,
             // Distance per revolution of absolute encoder
             ClimberConstants::kAbsoluteDistancePerRev,
             // Default velocity
             ClimberConstants::kDefaultVelocity,
             // Velocity scalar
             ClimberConstants::kVelocityScalar,
             // Tolerance
             ClimberConstants::kTolerance,
             // Min limit switch
             std::nullopt,
             // Max limit switch
             std::nullopt,
             // Reversed
             true,
             // Mechanism2d
             ClimberConstants::kClimberMechanism,
             // Conversion Function
             std::nullopt,

             [] { return false; }, ClimberConstants::kRotationalAxisConstraints}, 
             CoralArmConstants::kArmLength,
             node} {}

 private:

  rev::spark::SparkMax m_climberMotor{ClimberConstants::kArmMotorId,
                               rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkClosedLoopController m_pidController = m_climberMotor.GetClosedLoopController();
  rev::spark::SparkRelativeEncoder m_enc = m_climberMotor.GetEncoder();
  rev::spark::SparkAbsoluteEncoder m_absEnc = m_climberMotor.GetAbsoluteEncoder();
  subzero::PidSettings climberPidSettings = {
      ClimberConstants::kP, ClimberConstants::kI, ClimberConstants::kD,
      ClimberConstants::kIZone, ClimberConstants::kFF, true};
  SparkMaxPidController climberController{"Coral Arm",
                                   m_climberMotor,
                                   m_enc,
                                   m_pidController,
                                   climberPidSettings,
                                   &m_absEnc,
                                   ClimberConstants::kMaxRpm};
  subzero::SimPidMotorController simClimberController{"Sim Arm", climberPidSettings, ClimberConstants::kMaxRpm};
};