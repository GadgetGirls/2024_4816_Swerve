//

#pragma once

#include <string>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * Log a Pose2D object to the smart dashboard.
 *
 * @param pose The pose whose dimensions to log
 *  @return Success/Failure.
 */
void DisplayPose(std::string poseName, frc::Pose2d pose);
