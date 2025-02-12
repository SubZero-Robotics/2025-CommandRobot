
#include "subsystems/AlgaeArmSubsystem.h"

#include <frc2/command/RunCommand.h>

void AlgaeArmSubsystem::Periodic() {

}

// TODO: make the set value for in and out a constant

frc2::CommandPtr AlgaeArmSubsystem::StopIntake() {
   return
    frc2::InstantCommand([this] {                                                    \
             m_intakeMotor.Set(0);             
  }).ToPtr();
}


frc2::CommandPtr AlgaeArmSubsystem::In() {
    return
    frc2::RunCommand([this] {                                                    \
             m_intakeMotor.Set(0.25);             
  }, {this}).ToPtr();
}

frc2::CommandPtr AlgaeArmSubsystem::Out() {
    return
    frc2::InstantCommand([this] {                                                    \
             m_intakeMotor.Set(-0.25);             
  }).ToPtr();
}
