#pragma once

#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  // Subsystem method declarations go here.

  // Constructor
  IntakeSubsystem();

  // This method is called periodically by the CommandScheduler
  void Periodic() override;

  // Start intake rollers in the "in" direction
  void rollIn(double motorSpeed = 1.0);

  // Start intake rollers in the "out" direction
  void rollOut(double motorSpeed = 1.0);

  // Stop intake rollers
  void stopRollers();
  
  // Manually drive intake deploy/retract
  // void driveIntake(double speed);

  // Deploy intake assembly
  // void deployIntake();

  // Retract intake assembly
  // void retractIntake();

  // Deploy if retracted, retract if deployed
  // void toggleDeploy();

  // Run augers to feed balls from hopper to first stage of shooter
  void runAugers();

  // Stop augers from spinning
  void stopAugers();

  // Run hopper side winch until limit switch hits
  // void runHopperWinch();

  // Stop hopper side winch
  // void stopHopperWinch();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  
  // Intake roller motor
  rev::spark::SparkMax m_intakeRollerMotor{IntakeSubsystemConstants::kIntakeRollerCANId,
                                             rev::spark::SparkLowLevel::MotorType::kBrushless};
  // Intake deploy/retract motor
  // rev::spark::SparkFlex m_intakeDeployMotor{IntakeSubsystemConstants::kIntakeDeployCANId,
                                            // rev::spark::SparkLowLevel::MotorType::kBrushless};
  // Hopper auger motor
  rev::spark::SparkFlex m_intakeAugerMotor{IntakeSubsystemConstants::kIntakeAugerCANId,
                                             rev::spark::SparkLowLevel::MotorType::kBrushless};
  // Hopper side winch
  /*rev::spark::SparkMax m_intakeHopperMotor{IntakeSubsystemConstants::kIntakeHopperCANId,
                                              rev::spark::SparkLowLevel::MotorType::kBrushless};*/

  // Intake deployment limit switch - detect when intake is all the way down
  // frc::DigitalInput m_intakeDeployLimitSwitch{IntakeSubsystemConstants::kIntakeDeployLimitSwitchChannel};
  // Intake retract limit switch - detect when intake is all the way up
  //frc::DigitalInput m_intakeRetractLimitSwitch{IntakeSubsystemConstants::kIntakeRetractLimitSwitchChannel};

  // Hopper side limit switch
  // frc::DigitalInput m_intakeHopperLimitSwitch{IntakeSubsystemConstants::kIntakeHopperLimitSwitchChannel};

  bool m_isDeployed = false;  // Is the intake deployed? 

  bool m_rollerMotorOn = false;
  int m_rollerMotorDirection = -1; // -1 = IN, 1 = OUT, 0 = STOP (May need to flip IN and OUT)
  bool m_safeToRunAugers = false;

  frc::Timer m_augerTimer;
};