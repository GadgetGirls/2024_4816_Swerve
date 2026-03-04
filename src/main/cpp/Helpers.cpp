//

#include "Helpers.h"

bool DisplayPose(std::string poseName, frc::Pose2d pose){
    std::string x_label = poseName + "-X";
    std::string y_label = poseName + "-Y";
    std::string r_label = poseName + "-Rotation";
    frc::SmartDashboard::PutNumber(x_label, pose.X().value());
    frc::SmartDashboard::PutNumber(y_label, pose.Y().value());
    frc::SmartDashboard::PutNumber(r_label, pose.Rotation().Degrees().value());
}