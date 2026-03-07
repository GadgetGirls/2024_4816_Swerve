#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include "Constants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  void SetSpeed(double speed);

  void Stop();

  double GetSpeed();

  void SetSpeedForDistance(double distanceMeters);

  void SetFeederSpeed(double speed);

  void StopFeeder();

  void Shoot(double speed);
  
  frc2::CommandPtr ShootCommand(double speed);

  frc2::CommandPtr StopCommand();


  // This method is called periodically by the CommandScheduler
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  // Left stick controls shooter "out" (forward Y) and "in" (backward Y)
  // double m_operatorController.GetLeftY(	)

  // Shooter motor
  rev::spark::SparkMax m_shooterMotor{ShooterSubsystemConstants::kShooterCANId, 
                                    rev::spark::SparkLowLevel::MotorType::kBrushless};
  // Feeder motor
  rev::spark::SparkMax m_feederMotor{ShooterSubsystemConstants::kFeederCANId, 
                                    rev::spark::SparkLowLevel::MotorType::kBrushless};
};