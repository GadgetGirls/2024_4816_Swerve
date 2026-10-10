#include "subsystems/ShooterSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/REVLibError.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <stdio.h>
#include <iostream>


ShooterSubsystem::ShooterSubsystem() {
  SetName("Shooter");
}

void ShooterSubsystem::Periodic() {
  // Display current motor output on SmartDashboard
  frc::SmartDashboard::PutNumber("Shooter/Output", m_shooterMotor.Get());
}

void ShooterSubsystem::SetSpeed(double speed) {
  m_shooterMotor.Set(speed);
  m_secondShooterMotor.Set(-speed);  // 2nd motor follows first motor, inverted
}

void ShooterSubsystem::Stop() {
  StopFeeder();
  m_shooterMotor.Set(0.0);
  m_secondShooterMotor.Set(0.0);
}

double ShooterSubsystem::GetSpeed() {
  return m_shooterMotor.Get();
}

void ShooterSubsystem::SetSpeedForDistance(double distanceMeters) {
  // ================================================================
  // TUNE THESE VALUES THROUGH TESTING!
  // ================================================================
  // This is a simple example lookup table. You should:
  // 1. Test at various distances
  // 2. Record what speed successfully scores
  // 3. Update these thresholds and speeds
  //
  // For more precision, you could use linear interpolation between
  // known data points instead of discrete ranges.
  // ================================================================
  
  double speed = 0.0;
  
  if (distanceMeters < 0) {
    // Invalid distance (no target) - don't run shooter
    speed = 0.0;
    frc::SmartDashboard::PutString("Shooter/Status", "No Target");
  } 
  else if (distanceMeters < 2.0) {
    // Close range (under 2 meters) - lower speed
    speed = 0.5;
    frc::SmartDashboard::PutString("Shooter/Status", "Close Range");
  } 
  else if (distanceMeters < 3.5) {
    // Medium range (2-3.5 meters)
    speed = 0.65;
    frc::SmartDashboard::PutString("Shooter/Status", "Medium Range");
  } 
  else if (distanceMeters < 5.0) {
    // Long range (3.5-5 meters)
    speed = 0.8;
    frc::SmartDashboard::PutString("Shooter/Status", "Long Range");
  } 
  else {
    // Very far (over 5 meters) - full power
    speed = 1.0;
    frc::SmartDashboard::PutString("Shooter/Status", "Max Range");
  }
  
  frc::SmartDashboard::PutNumber("Shooter/TargetSpeed", speed);
  frc::SmartDashboard::PutNumber("Shooter/TargetDistance", distanceMeters);
  
  SetSpeed(speed);
}

void ShooterSubsystem::SetFeederSpeed(double speed){
  m_feederMotor.Set(speed);
}

void ShooterSubsystem::StopFeeder(){
  m_feederMotor.Set(0.0);
}

void ShooterSubsystem::Shoot() {
  SetFeederSpeed(ShooterSubsystemConstants::kFeederSpeed);
  SetSpeed(ShooterSubsystemConstants::kShooterSpeed);
}

frc2::CommandPtr ShooterSubsystem::ShootCommand() {
  // Returns a command that sets the shooter to a specific speed
  return this->RunOnce([this] { Shoot(); });
}

frc2::CommandPtr ShooterSubsystem::StopCommand() {
  // Returns a command that stops the shooter
  return this->RunOnce([this] { Stop(); });
}
