// Declare a Subsystem to operate the 2025 intake elevator

#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
    public:
        // Constructor
        ElevatorSubsystem();
        
        // Destructor
        ~ElevatorSubsystem();

        // Actions to take regularly as the robot runs
        void Periodic() override;
        
        /***
         * Methods that directly do a thing
         ***/
        // Need method to raise/lower (move!) elevator (positive == up, negative == down)
        void setSpeed(double speed);
        
        /***
         * Methods that return a CommandPtr to a thing that does the thing
         ***/
        frc2::CommandPtr runSetSpeed(double speed);

        // Possible future methods to move to preset positions: trough, first coral, 2nd coral, receive-from-human-player

    private:
        /***
         * Internal data to the subsystem
         */
        const int kElevatorLeftMotorCANId {9};
        const int kElevatorRightMotorCANId {12};
        const int kLimitSwitchChannel {9};
        
        /* Need 2 rev::Spark::SparkMax motors.
          These will need to be set so that one follows the other in reverse */
        rev::spark::SparkMax m_elevatorRaiseLowerMotor {kElevatorLeftMotorCANId, 
                                                    rev::spark::SparkLowLevel::MotorType::kBrushless};
        /* rev::spark::SparkMax m_elevatorRaiseLowerFollowerMotor{kElevatorRightMotorCANId, 
                                                    rev::spark::SparkLowLevel::MotorType::kBrushless};
        */
        // Need one or two Limit switches to tell us when the elevator needs to stop at the top/bottom
        frc::DigitalInput m_topLimitSwitch {kLimitSwitchChannel};

};