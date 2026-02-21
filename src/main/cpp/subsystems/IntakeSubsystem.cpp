#include <rev/SparkMax.h>
#include <rev/SparkLowLevel.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "RobotContainer.h"

IntakeSubsystem::IntakeSubsystem(){};

/*
2026 Intake subsystem includes:
- Intake arm to deploy and retract the intake (1 motor)
- Winch to bring hopper wall back during climbing (1 motor)
- Intake Augers (1 motor)
    - Own button control?
    - Or starts and ends with a lag when intake rollers run?
- Intake rollers (1 motor) - DONE
*/

void IntakeSubsystem::rollIn(double motorSpeed){
  // Start / stop intake rollers in the "in" direction
  //
  // If left bumper is pressed once, activate intake "in" direction
  // If left bumper is pressed again, stop intake "in" direction
  // REMEMBER: m_rollerMotorDirection : -1 = IN, 1 = OUT, 0 = STOP (May need to flip IN and OUT)
  // If we're reversing direction, we need to slow down, stop, and speed up in reverse
  if (m_rollerMotorOn == false) {
    m_rollerMotorDirection = -1;
    m_intakeRollerMotor.Set(m_rollerMotorDirection * motorSpeed);
    m_rollerMotorOn = true;
  } else {
    stopRollers();
  }
}


// Start intake rollers in the "out" direction
void IntakeSubsystem::rollOut(double motorSpeed){
  // If right bumper is pressed once, activate intake "out" direction
  // If right bumper is pressed once, stop intake "out" direction
  // REMEMBER: m_rollerMotorDirection : -1 = IN, 1 = OUT, 0 = STOP (May need to flip IN and OUT)
  // If we're reversing direction, we need to slow down, stop, and speed up in reverse
  if (m_rollerMotorOn == false) {
    m_rollerMotorDirection = 1;
    m_intakeRollerMotor.Set(m_rollerMotorDirection * motorSpeed);
    m_rollerMotorOn = true;
  } else {
    stopRollers();
  }
}

// Stop intake rollers
void IntakeSubsystem::stopRollers(){
  m_intakeRollerMotor.StopMotor();
  m_rollerMotorOn = false;
}

// Deploy intake
void IntakeSubsystem::deployIntake(){
  // Limit switches return false when closed/triggered.
  if (m_intakeDeployLimitSwitch.Get() == false){
      m_intakeDeployMotor.Set(0.0);
  } else {
      m_intakeDeployMotor.Set(kIntakeDeploySpeed);
  }
}

// Retract intake
void IntakeSubsystem::retractIntake(){
  // Limit switches return false when closed/triggered.
  if (m_intakeRetractLimitSwitch.Get() == false){
      m_intakeDeployMotor.Set(0.0);
  } else {
      m_intakeDeployMotor.Set(kIntakeRetractSpeed);
  }

}

// Run augers to feed balls from hopper to first stage of shooter
void IntakeSubsystem::runAugers(){
  m_intakeAugerMotor.Set(kIntakeAugerSpeed);
}

// Stop augers from spinning
void IntakeSubsystem::stopAugers(){
  m_intakeAugerMotor.Set(0.0);
}

// Run hopper side winch until limit switch hits
void IntakeSubsystem::runHopperWinch(){
  // Limit switches return false when closed/triggered.
  if (m_intakeHopperLimitSwitch.Get() == false){
      m_intakeHopperMotor.Set(0.0);
  } else {
      m_intakeHopperMotor.Set(kIntakeHopperSpeed);
  }
}

// Stop hopper side winch
void IntakeSubsystem::stopHopperWinch(){
  m_intakeHopperMotor.Set(0.0);
}

void IntakeSubsystem::Periodic(){}