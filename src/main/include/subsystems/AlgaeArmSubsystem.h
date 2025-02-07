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
                ? dynamic_cast<subzero::IPidMotorController&>(algaeArmController)
                : dynamic_cast<subzero::IPidMotorController&>(simAlgaeArmController),
            {
             AlgaeArmConstants::kHomeRotation,
             // Max distance
             AlgaeArmConstants::kMaxRotation,
             // Distance per revolution of relative encoder
             AlgaeArmConstants::kRelativeDistancePerRev,
             // Distance per revolution of absolute encoder
             AlgaeArmConstants::kAbsoluteDistancePerRev,
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
             AlgaeArmConstants::kAlgaeArmMechanism,
             // Conversion Function
             std::nullopt,

             [] { return false; }, AlgaeArmConstants::kRotationalAxisConstraints},
            AlgaeArmConstants::kArmLength,
            nullptr} {
    //m_Motor.SetIdleMode(rev::spark::SparkBase::IdleMode::kBrake);
    }
    
    void Periodic() override;

    void SimulationPeriodic() override;

    void Stop();

    void In(double);


    void Out(double);



  

 private:
  rev::spark::SparkMax m_intakeMotor{
      AlgaeArmConstants::kIntakeMotorId,
      rev::spark::SparkLowLevel::MotorType::kBrushless};

  rev::spark::SparkMax m_armMotor{AlgaeArmConstants::kArmMotorId,
                               rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkClosedLoopController m_pidController = m_armMotor.GetClosedLoopController();
  rev::spark::SparkRelativeEncoder m_enc = m_armMotor.GetEncoder();
  rev::spark::SparkAbsoluteEncoder m_absEnc = m_armMotor.GetAbsoluteEncoder();
  subzero::PidSettings algaeArmPidSettings = {
      AlgaeArmConstants::kP, AlgaeArmConstants::kI, AlgaeArmConstants::kD,
      AlgaeArmConstants::kIZone, AlgaeArmConstants::kFF};
  SparkMaxController algaeArmController{"Arm",
                                   m_armMotor,
                                   m_enc,
                                   m_pidController,
                                   algaeArmPidSettings,
                                   &m_absEnc,
                                   AlgaeArmConstants::kMaxRpm};
  subzero::SimPidMotorController simAlgaeArmController{"Sim Arm", algaeArmPidSettings,
                                                  AlgaeArmConstants::kMaxRpm};

};