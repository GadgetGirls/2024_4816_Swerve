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
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LEDSubsystem.h"
#include "subsystems/VisionSubsystem.h"
// #include "subsystems/ShooterSubsystem.h"

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
  frc2::Command* GetAutonomousCommand();
private:
 
  // Which alliance are we on?
  std::optional<frc::DriverStation::Alliance> m_alliance;

  // The driver's controller
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};

  // Joystick has inputs
  // GetTrigger()
  // GetRawButton(7, 8, 9, 10,11,12)

  // Trigger should run shooter in manual mode
  // frc2::Trigger m_driverTrigger = m_driverController.GetTrigger();

  // Button 10 should deploy the intake
  // frc2::Trigger m_driverButton10 = m_driverController.GetRawButton(10);


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

  // - Left stick up/down controls elevator

  // Left bumper controls intake roll "out" - 2026 hold to engage by default
  frc2::Trigger m_operatorLeftBumper = m_operatorController.LeftBumper();

  // Right bumper controls intake roll "in" - 2026 hold to engage by default
  frc2::Trigger m_operatorRightBumper = m_operatorController.RightBumper();


  // When set the robot goes at full throttle.  When clear full throttle is scaled down by
  // Should be k constants
  double button3_result;
  double throttle_percentage;
  bool fieldRelative;
  char controllerMode; // control whether bumpers toggle intake or are held to run intake

  // The robot's subsystems
  DriveSubsystem m_drive;
  ElevatorSubsystem m_elevator;
  IntakeSubsystem m_intake;
  LEDSubsystem m_led;
  VisionSubsystem m_vision;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;
  // Timer
  frc::Timer timer0;
  
  // Sample limelight return values
  double tx;
  double ty;
  double ta;
  double hasTarget;

  void ConfigureButtonBindings();
};