#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/geometry/Pose2d.h>
#include "Constants.h"
#include "LimelightHelpers.h"

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();

  bool HasTarget();

  double GetTX();
  
  double GetTY();
  
  int GetTargetID();

  std::optional<frc::Pose2d> GetBotPose();

  std::optional<frc::Pose3d> GetBotPose3d();

  double GetDistanceToTargetMeters();

  double GetLatencyMs();

  void Periodic();

  void SetPipeline(int pipelineIndex);
    
  void SetPriorityTagID(int tagID);

  frc::Pose2d GetTargetPose2d();

 private:
  std::string m_limelightName;

};
