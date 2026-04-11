#include "subsystems/VisionSubsystem.h"
#include <cmath>
#include <numbers>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <units/angle.h>
#include "LimelightHelpers.h"


VisionSubsystem::VisionSubsystem() {
  // If you have multiple Limelights, set the name here:
  m_limelightName = "";
  
  SetName("Vision");
}

void VisionSubsystem::Periodic() {
  // Publish key values to SmartDashboard for debugging
  bool hasTarget = HasTarget();
  frc::SmartDashboard::PutBoolean("Vision-HasTarget", hasTarget);
  
  double hasTarget_nt = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0);
  frc::SmartDashboard::PutNumber("NetTables-HasTarget", hasTarget_nt);

  if (hasTarget) {
    frc::SmartDashboard::PutNumber("Vision-TX", GetTX());
    frc::SmartDashboard::PutNumber("Vision-TY", GetTY());
    frc::SmartDashboard::PutNumber("Vision-TagID", GetTargetID());
    frc::SmartDashboard::PutNumber("Vision-Distance_m", GetDistanceToTargetMeters());
    frc::SmartDashboard::PutNumber("Vision-Latency_ms", GetLatencyMs());
    
    // Also publish bot pose if available
    /* auto pose = GetBotPose();
    if (pose.has_value()) {
      frc::SmartDashboard::PutNumber("Vision-BotPose_X", pose->X().value());
      frc::SmartDashboard::PutNumber("Vision-BotPose_Y", pose->Y().value());
      frc::SmartDashboard::PutNumber("Vision-BotPose_Rotation", pose->Rotation().Degrees().value());
    }
    */
  }
}

bool VisionSubsystem::HasTarget() {
  return LimelightHelpers::getTV(m_limelightName);
}

double VisionSubsystem::GetTX() {
  return LimelightHelpers::getTX(m_limelightName);
}

double VisionSubsystem::GetTY() {
  return LimelightHelpers::getTY(m_limelightName);
}

int VisionSubsystem::GetTargetID() {
  return static_cast<int>(LimelightHelpers::getFiducialID(m_limelightName));
}

void VisionSubsystem::SetTargetID(int targetTagID) {
  return LimelightHelpers::SetFiducialIDFiltersOverride(m_limelightName, std::vector<int>{targetTagID});
}

std::optional<frc::Pose2d> VisionSubsystem::GetBotPose() {
  if (!HasTarget()) {
    return std::nullopt;
  }
  
  // botpose_wpiblue gives pose relative to blue alliance origin
  // This is consistent regardless of which alliance you're on
  /* std::vector<double> poseData = LimelightHelpers::getBotpose_wpiBlue(m_limelightName);
  if (poseData.size() < 6) {
    return std::nullopt;
  }
  */
  LimelightHelpers::PoseEstimate poseData = LimelightHelpers::getBotPoseEstimate_wpiBlue(m_limelightName);	

  // return LimelightHelpers::toPose2D(poseData);
  return poseData.pose;
}

std::optional<frc::Pose3d> VisionSubsystem::GetBotPose3d() {
  
  if (!HasTarget()) {
    return std::nullopt;
  }
  
  /*
  std::vector<double> poseData = LimelightHelpers::getBotpose_wpiBlue(m_limelightName);
  if (poseData.has_value() and poseData.val.size() < 6) {
    return std::nullopt;
  }
  */
  frc::Pose3d poseData = LimelightHelpers::getBotPose3d_wpiBlue(m_limelightName);

  // return LimelightHelpers::toPose3D(poseData);
  return poseData;
}

double VisionSubsystem::GetDistanceToTargetMeters() {
  if (!HasTarget()) {
    return -1.0;  // Invalid distance indicator
  }
  
  // Distance calculation using trigonometry:
  //
  //   distance = (targetHeight - cameraHeight) / tan(cameraAngle + ty)
  //
  // Where:
  //   - targetHeight: Height of AprilTag center from floor (need both 44.25 and 21.75)
  //   - cameraHeight: Height of camera lens from floor
  //   - cameraAngle: Camera mount angle from horizontal
  //   - ty: Vertical angle to target (from Limelight) (in degrees)
  /* 
  
  double targetHeightInches
  double cameraHeightInches = 19 inches
  double cameraAngleDegrees = 35 degrees
     
  */
  
  double ty = GetTY();
  double angleToTargetRadians = (VisionSubsystemConstants::kLimelightMountAngleDegrees + ty) * (std::numbers::pi / 180.0);
  
  // Avoid divide by zero
  if (std::abs(angleToTargetRadians) < 0.001) {
    return -1.0;
  }
  
  double distance = (VisionSubsystemConstants::kTargetHeightMeters - VisionSubsystemConstants::kLimelightHeightMeters) / std::tan(angleToTargetRadians);
  
  // Sanity check - distance should be positive
  if (distance < 0) {
    return -1.0;
  }
  
  return distance;
}

double VisionSubsystem::GetLatencyMs() {
  // Total latency = pipeline latency + capture latency
  return LimelightHelpers::getLatency_Pipeline(m_limelightName) + 
         LimelightHelpers::getLatency_Capture(m_limelightName);
}

void VisionSubsystem::SetPipeline(int pipelineIndex) {
  LimelightHelpers::setPipelineIndex(m_limelightName, pipelineIndex);
}

void VisionSubsystem::SetPriorityTagID(int tagID) {
  LimelightHelpers::setPriorityTagID(m_limelightName, tagID);
}

// This returns a robot-relative pose that must be converted to field-relative
frc::Pose2d VisionSubsystem::GetTargetPose2d(){
  // Pose2d is a translation2d and a rotation2d
  const double targetDistance = GetDistanceToTargetMeters();
  units::degree_t x_angle{GetTX()};
  frc::Rotation2d targetRotation{x_angle};
  frc::Translation2d	targetTranslation{units::meter_t{targetDistance}, targetRotation};
  frc::Pose2d targetPose2d{targetTranslation, targetRotation};
  return(targetPose2d);
}

