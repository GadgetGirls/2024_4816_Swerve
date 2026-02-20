#pragma once

#include <frc2/command/SubsystemBase.h>
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
  
  // Deploy intake assembly
  void deployIntake();

  // Retract intake assembly
  void retractIntake();

  // Run augers to feed balls from hopper to first stage of shooter
  void runAugers();

  // Stop augers from spinning
  void stopAugers();


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  const int kIntakeRollerCANId = 9;
  const int kIntakeDeployCANId = 99; // CHANGEME
  const int kIntakeAugerCANId = 98; // CHANGEME

  // Intake roller motor
  rev::spark::SparkMax m_intakeRollerMotor{kIntakeRollerCANId,
                                             rev::spark::SparkLowLevel::MotorType::kBrushless};
  // Intake deploy/retract motor
  rev::spark::SparkMax m_intakeDeployMotor{kIntakeDeployCANId,
                                             rev::spark::SparkLowLevel::MotorType::kBrushless};
  // Hopper auger motor
  rev::spark::SparkMax m_intakeAugerMotor{kIntakeAugerCANId,
                                             rev::spark::SparkLowLevel::MotorType::kBrushless};

  static constexpr double k_rollerMotorSpeed = 1.0;

  bool m_rollerMotorOn = false;
  int m_rollerMotorDirection = -1; // -1 = IN, 1 = OUT, 0 = STOP (May need to flip IN and OUT)

};