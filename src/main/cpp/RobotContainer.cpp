// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "RobotContainer.h"

#include <frc/LEDPattern.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Subsystem.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/LEDSubsystem.h"
#include <frc/RobotController.h>

using namespace DriveConstants;
using namespace frc;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // Configure the button bindings
  ConfigureButtonBindings();
  timer0.Reset();
  // A few control variables
  fieldRelative=false;
  controllerMode='j'; // 'j' for josephine toggle bumpers, 'a' for avi hold bumpers

  // Initialize elevator and set to be controlled by Operator XBoxController Left stick
  m_elevator.SetDefaultCommand(frc2::RunCommand(
    [this] {
        double opctlr_left_y = -m_operatorController.GetLeftY() * 0.25;
        if (opctlr_left_y == 0){ opctlr_left_y = 0.2; } // elevator hold speed
        frc::SmartDashboard::PutNumber("OperatorCtlr LeftY", opctlr_left_y);
        m_elevator.setSpeed(opctlr_left_y);
    },
    {&m_elevator}
  ));
  
  // Initialize intake subsystem - nothing to do here right now
   /* m_intake.SetDefaultCommand(frc2::RunCommand(
    [this] {
        // Do stuff
    },
    {&m_intake}
  ));
  */

  // Set the LEDs to run Green
  m_led.SetDefaultCommand(m_led.RunPattern(frc::LEDPattern::Solid(ColorFlip(frc::Color::kGreen))));

  // Our LED strip has a density of 60 LEDs per meter
  units::meter_t kLedSpacing{1 / 60.0};

  // Create an LED pattern that will display a rainbow across
  // all hues at maximum saturation and half brightness
  LEDPattern m_rainbow = LEDPattern::Rainbow(255, 128);

  // Create a new pattern that scrolls the rainbow pattern across the LED
  // strip, moving at a speed of 1 meter per second.
  frc::LEDPattern m_scrollingRainbow = m_rainbow.ScrollAtAbsoluteSpeed(.5_mps, kLedSpacing);
  //m_scrollingRainbow = m_rainbow.Breathe(3_s);
  //m_scrollingRainbow = m_rainbow.Mask();

  // Create an LED pattern that displays a red-to-blue gradient.
// The LED strip will be red at both ends and blue in the center,
// with smooth gradients between
std::array<Color, 2> colors{ColorFlip(Color::kRed), ColorFlip(Color::kBlue)};
LEDPattern gradient = LEDPattern::Gradient(LEDPattern::GradientType::kDiscontinuous, colors);
std::array<std::pair<double, Color>, 2> colorSteps{std::pair{0.0, ColorFlip(Color::kRed)},
                                                   std::pair{0.5, Color::kBlue}};
LEDPattern steps = LEDPattern::Steps(colorSteps);

//std::array<Color, 2> colors{ColorFlip(Color::kRed), Color::kBlue};
//LEDPattern base = LEDPattern::LEDPattern::Gradient(LEDPattern::GradientType::kDiscontinuous, colors);
LEDPattern base = LEDPattern::Steps(colorSteps);
LEDPattern pattern = base.ScrollAtRelativeSpeed(units::hertz_t{0.25});
LEDPattern absolute = base.ScrollAtAbsoluteSpeed(0.125_mps, units::meter_t{1/120.0});
//LEDPattern sycned = base.SynchronizedBlink([]() { return RobotController::IsSysActive(); });
LEDPattern sycned = base.SynchronizedBlink([]() { return RobotController::GetRSLState(); });
//m_led.SetDefaultCommand(m_led.RunPattern(sycned));

m_led.SetDefaultCommand(frc2::RunCommand(
    [this] {
        SmartDashboard::PutNumber("preThrottle",button3_result);
        button3_result = m_driverController.GetThrottle();
        button3_result--;
        button3_result = button3_result * -1;
        SmartDashboard::PutNumber("Throttle",button3_result);

        std::array<Color, 2> colors{ColorFlip(Color::kRed), ColorFlip(Color::kBlue)};
        LEDPattern gradient = LEDPattern::Gradient(LEDPattern::GradientType::kDiscontinuous, colors);
        gradient = gradient.AtBrightness(button3_result);
        m_led.ApplyPattern(gradient);
    },
    {&m_led}));    
      
  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        // GetThrottle returns an analog value from -1 to 1. We need to transform that to a percentage
        button3_result = m_driverController.GetThrottle();
        // frc::SmartDashboard::PutNumber("Throttle", button3_result);
        // When the throttle is all the way towards the top the result is -1
        // The kids want this to be 100%.  So subtract one to get a value between
        // 0 and -2.  Then multiply by -1 to get rid of the negative and divide 
        // by 2 to get a value between 0 and 1.  This is our percentage.
        button3_result--;
        button3_result = button3_result * -1;
        // frc::SmartDashboard::PutNumber("Adjusted Throttle", button3_result);
        throttle_percentage = button3_result * 0.5;
        SmartDashboard::PutNumber("Throttle2",throttle_percentage);

        // Below a certain percentage the robot won't move at all.  Don't
        // let the throttle below this value.
        if (throttle_percentage < 0.15) {throttle_percentage = 0.15;}
        // Robot is uncontrollable at full throttle so max at 90 percent
        else if (throttle_percentage > 0.9) {throttle_percentage = 0.9;}
        frc::SmartDashboard::PutNumber("Throttle percentage", throttle_percentage);
        
        // Pushing buttons 11 and 12 resets the Z axis heading.  This could
        // be useful if the gyro drifts a lot
        //swapped 7 & 8 with 11 & 12
        if (m_driverController.GetRawButtonPressed(7) && m_driverController.GetRawButtonPressed(8))
            { m_drive.ZeroHeading();} 
        
        if (m_driverController.GetRawButtonPressed(11) && m_driverController.GetRawButtonPressed(12))
            { fieldRelative=!fieldRelative;}

        // NOTE: getY() reversed to deal with directional issue
        frc::SmartDashboard::PutNumber("Field Relative", fieldRelative);
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetY()  * throttle_percentage, OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetX() * throttle_percentage , OIConstants::kDriveDeadband)},    
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetTwist() * throttle_percentage, OIConstants::kDriveDeadband)},
            fieldRelative);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {  
  // Start / stop intake rollers in the "in" direction
  // OnTrue args should be Command - convert m_intake.rollIn() to command created by StartEnd?
  m_operatorController.LeftBumper().OnTrue(m_intake.RunOnce(
    [this] {
        m_intake.rollOut(1.0);
    }
  ));
  if(controllerMode == 'a'){
    m_operatorController.LeftBumper().OnFalse(m_intake.RunOnce(
      [this] {
        m_intake.rollOut(1.0);
      }
    ));
  }

  // Start / stop intake rollers in the "out" direction
  m_operatorController.RightBumper().OnTrue(m_intake.RunOnce(
    [this] {
        m_intake.rollIn(1.0);
    }
  ));
  if(controllerMode == 'a'){
    m_operatorController.RightBumper().OnFalse(m_intake.RunOnce(
        [this] {
            m_intake.rollIn(1.0);
        }
    ));
  }
  // For loading, use the Triggers at 1/2 speed
  m_operatorController.LeftTrigger().OnTrue(m_intake.RunOnce(
    [this] {
        m_intake.rollOut(0.5);
    }
  ));
  if(controllerMode == 'a'){
    m_operatorController.LeftTrigger().OnFalse(m_intake.RunOnce(
        [this] {
            m_intake.rollOut(0.5);
        }
        ));
  }
  m_operatorController.RightTrigger().OnTrue(m_intake.RunOnce(
    [this] {
        m_intake.rollIn(0.5);
    }
  ));
  if(controllerMode == 'a'){
    m_operatorController.RightTrigger().OnFalse(m_intake.RunOnce(
        [this] {
            m_intake.rollIn(0.5);
        }
    ));
  }  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    /*
    // Raise the elevator for 2 seconds
    m_elevator.m_elevatorTimer.Start();
    while(m_elevator.m_elevatorTimer.Get() <  two_seconds){
      m_elevator.setSpeed(1);
    }
    m_elevator.setSpeed(0);
    */

  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed/2,
                               AutoConstants::kMaxAcceleration/2);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);
  // https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1_trajectory_generator.html
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // waypoint 
      //WF- Keeping this example waypoint code in case we need to use something like
      // this in the future.  It is completely useless for now since we want to move in a 
      // straight line
      {frc::Translation2d{1_m, 0_m},
      frc::Translation2d{2_m, 0_m}},
      // {},  // No internal waypoints (empty vector)
      // Endpoint 1 meter from where we started.  This is enough distance to exit the starting zone and 
      // earn some points
      frc::Pose2d{3_m, 0_m, 0_deg}, 
      // Testing pose (short distance) = 1_m, 0_m, 0_deg
      // Josephine & Will's numbers = 3_m, 0_m, 0_deg
      // Phil's numbers based on Game Manual = 5.87_m, 0_m, 0_deg
      
      config);
  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};
  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});
  // https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc2_1_1_swerve_controller_command.html
  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, 
      
      [this]() { return m_drive.GetPose(); },
      m_drive.kDriveKinematics,
      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, 
      thetaController,
      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },
      {&m_drive});
  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());
  // Run swerveControllerCommand above to drive the trajectory, 
  // then run InstantCommand to stop
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(3_mps, 3_mps, 0_rad_per_s, false); }), 
      frc2::InstantCommand(
        [this]() { m_elevator.autoRaise();}),
      frc2::InstantCommand(
          [this]() { m_intake.rollOut(); })
  );
}
