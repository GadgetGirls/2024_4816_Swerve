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
#include <units/velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/LEDSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  timer0.Reset();
  fieldRelative=false;

  // Initialize elevator and set to be controlled by Operator XBoxController Left stick
  m_elevator.SetDefaultCommand(frc2::RunCommand(
    [this] {
        double opctlr_left_y = -m_operatorController.GetLeftY();
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
        // Below a certain percentage the robot won't move at all.  Don't
        // let the throttle below this value.
        if (throttle_percentage < 0.15) {throttle_percentage = 0.15;}
        // Robot is uncontrollable at full throttle so max at 90 percent
        else if (throttle_percentage > 0.9) {throttle_percentage = 0.9;}
        frc::SmartDashboard::PutNumber("Throttle percentage", throttle_percentage);
        
        // Pushing buttons 11 and 12 resets the Z axis heading.  This could
        // be useful if the gyro drifts a lot
        if (m_driverController.GetRawButton(11) && m_driverController.GetRawButton(12))
            { m_drive.ZeroHeading();} 
        
        if (m_driverController.GetRawButton(7) && m_driverController.GetRawButton(8))
            { fieldRelative=!fieldRelative;} //fix me maybe { m_drive.fieldRelative();}


        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetX()  * throttle_percentage, OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetY() * throttle_percentage , OIConstants::kDriveDeadband)},    
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetTwist() * throttle_percentage, OIConstants::kDriveDeadband)},
            true);
      },
      {&m_drive}));

      std::string default_str = std::string("Default");
      std::string right_str = std::string("Right");
      std::string middle_str = std::string("Middle");
      std::string left_str = std::string("Left");

      // Set up the autonomous mode chooser
      m_chooser.SetDefaultOption("Default Auto", &default_str);
      m_chooser.AddOption("Right Auto", &right_str);
      m_chooser.AddOption("Middle Auto", &middle_str);
      m_chooser.AddOption("Left Auto", &left_str);
    
      // Put the chooser on the dashboard
      frc::SmartDashboard::PutData(&m_chooser);
}

void RobotContainer::ConfigureButtonBindings() {  
  // Start / stop intake rollers in the "in" direction
  // OnTrue args should be Command - convert m_intake.rollIn() to command created by StartEnd?
  m_operatorController.LeftBumper().OnTrue(m_intake.RunOnce(
    [this] {
        m_intake.rollIn();
    }
  ));

  // Start / stop intake rollers in the "out" direction
  m_operatorController.RightBumper().OnTrue(m_intake.RunOnce(
    [this] {
        m_intake.rollOut();
    }
  ));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {

    // Set up config for trajectory
    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed/2,
        AutoConstants::kMaxAcceleration/2);

    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(m_drive.kDriveKinematics);
    // Do we need to regulate voltage here, too? ^^^

    // Choose the autonomous trajectory - left, middle, right, or default
    std::string* trajectory_name = m_chooser.GetSelected();

    // Choose trajectory
    if (*trajectory_name == "Left"){
        // No op
    } else if (*trajectory_name == "Middle"){
        // No op
    } else if (*trajectory_name == "Right"){
        // no op
    } else {  // "Default"
        // https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1_trajectory_generator.html
        m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            // Start at the origin facing the +X direction
            frc::Pose2d{0_m, 0_m, 0_deg},
        
            // waypoint 
            /* WF- Keeping this example waypoint code in case we need to use something like
            // this in the future.  It is completely useless for now since we want to move in a 
            // straight line
            {frc::Translation2d{2_m, 0_m},
            frc::Translation2d{4_m, 0_m}}, */
            {},  // No internal waypoints (empty vector)
        
            // Endpoint 1 meter from where we started.  This is enough distance to exit the starting zone and 
            // earn some points
            // Starting line to reef is 224 cm = 2.24m
            frc::Pose2d{2.24_m, 0_m, 0_deg}, 
            // 5.87_m, 0_m, 0_deg - I think this number was inflated to achieve a real-world distance. We should determine scaling factor.
            // 3_m, 1_m, 0_deg
            // 3_m, 0_m, -1_deg = no swerving, just forward
        
            // Pass the config
            config
        );
    }

    frc::ProfiledPIDController<units::radians> thetaController{AutoConstants::kPThetaController, 0, 0, AutoConstants::kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                    units::radian_t{std::numbers::pi});

    // https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc2_1_1_swerve_controller_command.html
    frc2::SwerveControllerCommand<4> swerveControllerCommand(
        m_trajectory, 

        [this]() { return m_drive.GetPose(); },

        m_drive.kDriveKinematics,

        frc::PIDController{AutoConstants::kPXController, 0, 0},
        frc::PIDController{AutoConstants::kPYController, 0, 0}, 
        thetaController,

        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

        {&m_drive}
    );

    // Reset odometry to the starting pose of the trajectory.
    m_drive.ResetOdometry(m_trajectory.InitialPose());

    // Run swerveControllerCommand above to drive the trajectory, 
    // then run InstantCommand to stop
    return new frc2::SequentialCommandGroup(
        std::move(swerveControllerCommand),
        frc2::InstantCommand(
            [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {})
        );
}


