// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "RobotContainer.h"
#include <chrono>
#include <thread>
#include <frc/DriverStation.h>
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
#include <networktables/NetworkTable.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <utility>

#include "Constants.h"
#include "LimelightHelpers.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/LEDSubsystem.h"
#include <frc/RobotController.h>
#include "LimelightHelpers.h"

using namespace DriveConstants;
using namespace frc;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Determine alliance - used to determine which AprilTags are our hub
  m_alliance = frc::DriverStation::GetAlliance();
  if (m_alliance == frc::DriverStation::Alliance::kBlue){
      frc::SmartDashboard::PutString("Our Alliance is ", "Blue");    
      m_hubAprilTagID = 26;  // or 25
      m_towerAprilTagID = 31;  // or 32
  } else {
      frc::SmartDashboard::PutString("Our Alliance is ", "Red");
      m_hubAprilTagID = 10;  // or 9
      m_towerAprilTagID = 15;  // or 16
    }
  // AprilTagFieldLayout.loadField(AprilTagFields.FRC_2026)
  m_vision.SetTargetID(m_hubAprilTagID);  // Start by looking for the hub

  // Configure the button bindings
  ConfigureButtonBindings();
  timer0.Reset();
  // A few control variables
  fieldRelative=false;
  controllerMode='a'; // 'j' for josephine toggle bumpers, 'a' for avi hold bumpers

  // Initialize elevator and set to be controlled by Operator XBoxController Right stick
  m_elevator.SetDefaultCommand(frc2::RunCommand(
    [this] {
        double opctlr_right_y = -m_operatorController.GetRightY() * 0.25;
        frc::SmartDashboard::PutNumber("OperatorCtlr RightY", opctlr_right_y);
        m_elevator.setSpeed(opctlr_right_y);
    },
    {&m_elevator}
  ));
  
  // Initialize intake subsystem - could put auger duty cycle here instead of periodic
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
    //LEDPattern synced = base.SynchronizedBlink([]() { return RobotController::IsSysActive(); });
    LEDPattern synced = base.SynchronizedBlink([]() { return RobotController::GetRSLState(); });
    //m_led.SetDefaultCommand(m_led.RunPattern(synced));
    /*
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
    */   

  // Set Vision subsystem default command
  m_vision.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_vision.Periodic();
    },
    {&m_vision}
  ));

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        // Can put call to get limelight position and target position data here
        // std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
        // float tx = table->GetNumber("tx");

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
        
        // Pushing buttons 7 and 8 resets the Z axis heading.  This could
        // be useful if the gyro drifts a lot
        // Pushing buttons 11 & 12 turns fieldRelative on or off <<< DISABLED ELSEWHERE
        if (m_driverController.GetRawButtonPressed(7) && m_driverController.GetRawButtonPressed(8))
            { m_drive.ZeroHeading();}
        /*
        if (m_driverController.GetRawButtonPressed(11) && m_driverController.GetRawButtonPressed(12))
            { fieldRelative=!fieldRelative;}
        */

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

frc2::Command* RobotContainer::AimDriveAndShoot(){
    // Set target AprilTag to hub tag
    m_vision.SetTargetID(m_hubAprilTagID);
    // Get our current location
    frc::Pose2d currentPose2D = m_drive.GetPose();
    // Get the target location
    frc::Pose2d targetPose2D = m_vision.GetTargetPose2d();
    // Backoff the target location far enough to shoot
    targetPose2D = ApplyBackoff(targetPose2D, kTargetBackoffDistance);
    // Set up config for trajectory
    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed/2,
                                  AutoConstants::kMaxAcceleration/2);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(m_drive.kDriveKinematics);
    // Create the trajectory
    // https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1_trajectory_generator.html
    auto ourTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at current position
      // frc::Pose2d{0_m, 0_m, 0_deg},
      currentPose2D,
      {},  // No internal waypoints (empty vector)
      targetPose2D,
      config);

    // Call drive subsystem to get there
    frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                          units::radian_t{std::numbers::pi});

    // https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc2_1_1_swerve_controller_command.html
    frc2::SwerveControllerCommand<4> swerveControllerCommand(
      ourTrajectory, 
      [this]() { return m_drive.GetPose(); },
      m_drive.kDriveKinematics,
      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, 
      thetaController,
      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },
      {&m_drive});

    // Reset odometry to the starting pose of the trajectory.
    m_drive.ResetOdometry(ourTrajectory.InitialPose());
    // Run swerveControllerCommand above to drive the trajectory, 
    // then run InstantCommand to stop
    return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }),
      frc2::InstantCommand(
        [this](){ m_shooter.Shoot(0.75); }
      )
    );
}

void RobotContainer::ScanForAprilTag(int tagNumber){ // CODING HERE
  // Swivel in a 270 degree arc looking for the AprilTag
  // Stop when you get a tag
  for(int i = 0; i < 270; i += 15){
    if (m_vision.HasTarget()){
      return;
    };
    // Turn i degrees
    m_drive.Drive(units::meters_per_second_t{0},
             units::meters_per_second_t{0}, 
             units::radians_per_second_t{2.365}, // 270 degrees in 2 seconds
             this->fieldRelative);
   // 15 degrees at 270 degrees/2 seconds is .111 seconds
   std::this_thread::sleep_for(std::chrono::milliseconds(111)); 
  };
}

void RobotContainer::ConfigureButtonBindings() {  
  // Start / stop intake rollers in the "in" direction
  // OnTrue args should be Command - convert m_intake.rollIn() to command created by RunOnce()
  m_operatorController.LeftBumper().OnTrue(m_intake.RunOnce(
    [this] {
        m_intake.rollIn(1.0);
    }
  ));
  if(controllerMode == 'a'){
    m_operatorController.LeftBumper().OnFalse(m_intake.RunOnce(
      [this] {
        m_intake.rollIn(1.0);
      }
    ));
  }

  // Start / stop intake rollers in the "out" direction
  m_operatorController.RightBumper().OnTrue(m_intake.RunOnce(
    [this] {
        m_intake.rollOut(1.0);
    }
  ));
  if(controllerMode == 'a'){
    m_operatorController.RightBumper().OnFalse(m_intake.RunOnce(
        [this] {
            m_intake.rollOut(1.0);
        }
    ));
  }

  // Operator controller right stick moves elevator in manual mode

  // Joystick Trigger should run shooter in manual mode
  m_joystickTrigger.OnTrue(m_shooter.RunOnce(
    [this] {
      // Start augers and feeder
      // m_intake.runAugers();  // Augers on duty cycle
      m_shooter.SetFeederSpeed(1.0); // CHANGEME
      // m_shooter.SetSpeed(1.0);  // Shooter motor runs constantly
    }
  ));
    m_joystickTrigger.OnFalse(m_shooter.RunOnce(
    [this] {
      // Start augers and feeder
      // m_shooter.SetSpeed(0.0);  // Shooter motor runs constantly
      m_shooter.SetFeederSpeed(0.0);
      // m_intake.stopAugers();  // Augers run on duty cycle
    }
  ));

  // Joystick Button 10 should deploy/retract the intake
  m_driverButton10.OnTrue(m_intake.RunOnce(
    [this]{
      m_intake.toggleDeploy();
    }
  ));

  // Joystick button 2 is auto-aim and shoot
  m_driverButton2.OnTrue(AimDriveAndShoot());

}


// Calculate a new target pose with backoff distance
frc::Pose2d RobotContainer::ApplyBackoff(frc::Pose2d targetPose, double distance){
  const Rotation2d& rotation = targetPose.Rotation();
  double x = rotation.Cos() * distance;
  double y = rotation.Sin() * distance;
  frc::Transform2d backoff = Transform2d(units::meter_t{x}, units::meter_t{y}, rotation);
  return targetPose + backoff;
}


frc2::Command* RobotContainer::GetAutonomousCommand() {
    
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed/2,
                               AutoConstants::kMaxAcceleration/2);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // Check m_vision.HasTarget();
  // If it's FALSE, go on a search for AprilTags
  
  // Get target pose
  frc::Pose2d targetPose2d = m_vision.GetTargetPose2d();  // Needs tag ID
  // Offset this from the AprilTag position for shooting
  targetPose2d = ApplyBackoff(targetPose2d, kTargetBackoffDistance);

  // https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1_trajectory_generator.html
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // waypoint 
      {frc::Translation2d{2_m, 0_m},
        frc::Translation2d{0_m, 180_deg}},
      // {},  // No internal waypoints (empty vector)
      // frc::Pose2d{3_m, 0_m, 0_deg}, 
      // Testing pose (short distance) = 1_m, 0_m, 0_deg
      // Josephine & Will's numbers = 3_m, 0_m, 0_deg
      targetPose2d,
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
  /* Run swerveControllerCommand above to drive the trajectory, 
     then run InstantCommand to stop

     Old default autonomous drive command:
     frc2::InstantCommand(
          [this]() { m_drive.Drive(3_mps, 3_mps, 0_rad_per_s, false); }),
  */
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { ScanForAprilTag(m_hubAprilTagID); }),  // Sweep scan for april tag
      frc2::InstantCommand(
          [this]() { m_intake.rollOut(); })
  );
}
