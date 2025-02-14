#pragma once

#include <frc/RobotBase.h>
#include <frc/smartdashboard/Mechanism2d.h>

#include <subzero/motor/IPidMotorController.h>
#include <subzero/motor/PidMotorController.h>
#include <subzero/motor/SimPidMotorController.h>
#include <subzero/singleaxis/RotationalSingleAxisSubsystem.h>

#include "Constants.h"
#include "subsystems/IntakeSubsystem.h"

//pretty much a copy paste of algae arm subsystem with differet names right now, subject to change later

class CoralArmSubsystem : public subzero::RotationalSingleAxisSubsystem<subzero::IPidMotorController> {
 public:
  explicit CoralArmSubsystem(frc::MechanismObject2d* node = nullptr)
      : subzero::RotationalSingleAxisSubsystem<subzero::IPidMotorController>{
            "Coral Arm",
            frc::RobotBase::IsReal()
                ? dynamic_cast<subzero::IPidMotorController&>(coralArmController)
                : dynamic_cast<subzero::IPidMotorController&>(simCoralArmController),
            {
             CoralArmConstants::kHomeRotation,
             // Max distance
             CoralArmConstants::kMaxRotation,
             // Distance per revolution of relative encoder
             CoralArmConstants::kRelativeDistancePerRev,
             // Distance per revolution of absolute encoder
             CoralArmConstants::kAbsoluteDistancePerRev,
             // Default velocity
             CoralArmConstants::kDefaultVelocity,
             // Velocity scalar
             CoralArmConstants::kVelocityScalar,
             // Tolerance
             CoralArmConstants::kTolerance,
             // Min limit switch
             std::nullopt,
             // Max limit switch
             std::nullopt,
             // Reversed
             true,
             // Mechanism2d
             CoralArmConstants::kCoralArmMechanism,
             // Conversion Function
             std::nullopt,

             [] { return false; }, CoralArmConstants::kRotationalAxisConstraints},
            CoralArmConstants::kArmLength,
            node} {
    //m_Motor.SetIdleMode(rev::spark::SparkBase::IdleMode::kBrake);
    }
    

    frc2::CommandPtr MoveAtSpeed(double speed);

    IntakeSubsystem& GetIntakeSubsystem();

 private:
  IntakeSubsystem m_intake{CoralArmConstants::kIntakeMotorId};

  rev::spark::SparkMax m_coralArmMotor{CoralArmConstants::kArmMotorId,
                               rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkClosedLoopController m_pidController = m_coralArmMotor.GetClosedLoopController();
  rev::spark::SparkRelativeEncoder m_enc = m_coralArmMotor.GetEncoder();
  rev::spark::SparkAbsoluteEncoder m_absEnc = m_coralArmMotor.GetAbsoluteEncoder();
  subzero::PidSettings coralArmPidSettings = {
      CoralArmConstants::kP, CoralArmConstants::kI, CoralArmConstants::kD,
      CoralArmConstants::kIZone, CoralArmConstants::kFF, true};
  SparkMaxPidController coralArmController{"Coral Arm",
                                   m_coralArmMotor,
                                   m_enc,
                                   m_pidController,
                                   coralArmPidSettings,
                                   &m_absEnc,
                                   CoralArmConstants::kMaxRpm};
  subzero::SimPidMotorController simCoralArmController{"Sim Arm", coralArmPidSettings,
                                                  CoralArmConstants::kMaxRpm};

};