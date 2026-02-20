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
  // How do we know when to stop? Is there a limit switch or sensor?
  // And is this a motor or more like a solenoid that releases
}

// Retract intake
void IntakeSubsystem::retractIntake(){
  // How do we know when to stop? Is there a limit switch or sensor?
  // Or do we need to read current draw from the motor to know it's working harder?
}

// Run augers to feed balls from hopper to first stage of shooter
void runAugers(){
  
}

// Stop augers from spinning
void stopAugers(){

}

void IntakeSubsystem::Periodic(){}