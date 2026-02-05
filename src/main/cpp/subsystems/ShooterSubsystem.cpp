#include "subsystems/ShooterSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ShooterSubsystem::ShooterSubsystem() {
  SetName("Shooter");
  
  // Configure motor settings as needed
  // m_shooterMotor.SetInverted(false);
  
  // You might want to set current limits for safety:
  // m_shooterMotor.SetSmartCurrentLimit(40);  // 40 amp limit
}

void ShooterSubsystem::Periodic() {
  // Display current motor output on SmartDashboard
  frc::SmartDashboard::PutNumber("Shooter/Output", m_shooterMotor.Get());
}

void ShooterSubsystem::SetSpeed(double speed) {
  m_shooterMotor.Set(speed);
}

void ShooterSubsystem::Stop() {
  m_shooterMotor.Set(0.0);
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

frc2::CommandPtr ShooterSubsystem::ShootCommand(double speed) {
  // Returns a command that sets the shooter to a specific speed
  return this->RunOnce([this, speed] { SetSpeed(speed); });
}

frc2::CommandPtr ShooterSubsystem::StopCommand() {
  // Returns a command that stops the shooter
  return this->RunOnce([this] { Stop(); });
}
