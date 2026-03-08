// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LEDSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  frc::Pose2d ApplyBackoff(frc::Pose2d targetPose, double distance);
  frc2::Command* GetAutonomousCommand();
  // frc2::Command* AimDriveAndShoot();
  void ScanForAprilTag(int tagNumber);
  double ApplyDeadband(double value);
  
private:
 
  // Which alliance are we on?
  std::optional<frc::DriverStation::Alliance> m_alliance;

  // The driver's controller
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};

  // Joystick has inputs
  // GetTrigger()
  // GetRawButton(7, 8, 9, 10,11,12)

  // Trigger should run shooter in manual mode
  // frc2::JoystickButton m_joystickTrigger{&m_driverController, frc::Joystick::ButtonType::kTriggerButton};
  //frc2::JoystickButton m_joystickTrigger{&m_driverController, 1}; // Former attempt

  // Button 2 should aim, position, and fire automatically
  frc2::JoystickButton m_driverButton2{&m_driverController, 2};

  // Pushing buttons 7 and 8 resets the Z axis heading.  This could
  // be useful if the gyro drifts a lot

  // Button 10 should deploy/retract the intake
  frc2::JoystickButton m_driverButton10{&m_driverController, 10};

  // Pushing buttons 11 & 12 turns fieldRelative on or off <<< DISABLED ELSEWHERE


  // The operator's controller:
  // Available XboxController buttons
  // A button
  // B button
  // X button
  // Y button
  // LeftBumper
  // RightBumper
  // LeftTrigger
  // RightTrigger

  frc2::CommandXboxController m_operatorController{OIConstants::kOperatorControllerPort};

  // Left stick controls intake deploy/retract

  // - Right stick up/down controls elevator

  // Left bumper controls intake roll "out" - 2026 hold to engage by default
  frc2::Trigger m_operatorLeftBumper = m_operatorController.LeftBumper();

  // Right bumper controls intake roll "in" - 2026 hold to engage by default
  frc2::Trigger m_operatorRightBumper = m_operatorController.RightBumper();


  // When set the robot goes at full throttle.  When clear full throttle is scaled down by ??? - Will
  //
  double button3_result;
  double throttle_percentage;

  bool fieldRelative;
  char controllerMode; // control whether bumpers toggle intake or are held to run intake
  int m_hubAprilTagID;
  int m_towerAprilTagID;

  // The robot's subsystems
  DriveSubsystem m_drive;
  ElevatorSubsystem m_elevator;
  IntakeSubsystem m_intake;
  LEDSubsystem m_led;
  ShooterSubsystem m_shooter;
  // VisionSubsystem m_vision;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;
  // Timer
  frc::Timer timer0;
  
  // Sample limelight return values
  double tx;
  double ty;
  double ta;
  double hasTarget;

  // Test constants
  double kTargetBackoffDistance{1};  // CHANGEME

  void ConfigureButtonBindings();
};