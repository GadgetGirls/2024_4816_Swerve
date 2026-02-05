#include "subsystems/VisionSubsystem.h"
#include <cmath>
#include <numbers>

VisionSubsystem::VisionSubsystem() {
  // If you have multiple Limelights, set the name here:
  // m_limelightName = "limelight-front";
  
  SetName("Vision");
}

void VisionSubsystem::Periodic() {
  // Publish key values to SmartDashboard for debugging
  bool hasTarget = HasTarget();
  
  frc::SmartDashboard::PutBoolean("Vision/HasTarget", hasTarget);
  
  if (hasTarget) {
    frc::SmartDashboard::PutNumber("Vision/TX", GetTX());
    frc::SmartDashboard::PutNumber("Vision/TY", GetTY());
    frc::SmartDashboard::PutNumber("Vision/TagID", GetTargetID());
    frc::SmartDashboard::PutNumber("Vision/Distance_m", GetDistanceToTargetMeters());
    frc::SmartDashboard::PutNumber("Vision/Latency_ms", GetLatencyMs());
    
    // Also publish bot pose if available
    auto pose = GetBotPose();
    if (pose.has_value()) {
      frc::SmartDashboard::PutNumber("Vision/BotPose_X", pose->X().value());
      frc::SmartDashboard::PutNumber("Vision/BotPose_Y", pose->Y().value());
      frc::SmartDashboard::PutNumber("Vision/BotPose_Rotation", pose->Rotation().Degrees().value());
    }
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

std::optional<frc::Pose2d> VisionSubsystem::GetBotPose() {
  if (!HasTarget()) {
    return std::nullopt;
  }
  
  // botpose_wpiblue gives pose relative to blue alliance origin
  // This is consistent regardless of which alliance you're on
  std::vector<double> poseData = LimelightHelpers::getBotpose_wpiBlue(m_limelightName);
  
  if (poseData.size() < 6) {
    return std::nullopt;
  }
  
  return LimelightHelpers::toPose2D(poseData);
}

std::optional<frc::Pose3d> VisionSubsystem::GetBotPose3d() {
  if (!HasTarget()) {
    return std::nullopt;
  }
  
  std::vector<double> poseData = LimelightHelpers::getBotpose_wpiBlue(m_limelightName);
  
  if (poseData.size() < 6) {
    return std::nullopt;
  }
  
  return LimelightHelpers::toPose3D(poseData);
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
  //   - targetHeight: Height of AprilTag center from floor
  //   - cameraHeight: Height of camera lens from floor
  //   - cameraAngle: Camera mount angle from horizontal
  //   - ty: Vertical angle to target (from Limelight)
  
  double ty = GetTY();
  double angleToTargetRadians = (kLimelightMountAngleDegrees + ty) * (std::numbers::pi / 180.0);
  
  // Avoid divide by zero
  if (std::abs(angleToTargetRadians) < 0.001) {
    return -1.0;
  }
  
  double distance = (kTargetHeightMeters - kLimelightHeightMeters) / std::tan(angleToTargetRadians);
  
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
