#include "subsystems/ElevatorSubsystem.h"
#include "Constants.h"
#include "RobotContainer.h"

// Define our ElevatorSubsystem Constructor
ElevatorSubsystem::ElevatorSubsystem(){
    m_elevatorRaiseLowerMotor.Set(0);
};

// The Linker says we have to define a non-virtual destructor function
ElevatorSubsystem::~ElevatorSubsystem(){
    m_elevatorRaiseLowerMotor.Set(0); // Stop the motor before we clean up
    // Could optionally explicitly delete m_elevatorRaiseLowerMotor and m_topLimitSwitch here
};

// Do periodic subsystem things here. 
void ElevatorSubsystem::Periodic(){
    // Perhaps update speed based on controller?
};

// Set the Elevator motor speeds to raise or lower
void ElevatorSubsystem::setSpeed(double speed){
        if (speed > 0){
                if (m_topLimitSwitch.Get() == true){
                        m_elevatorRaiseLowerMotor.Set(0);
                } else {
                        m_elevatorRaiseLowerMotor.Set(speed);
                }
        } else {
                // Will we have a bottom limitSwitch too?
                m_elevatorRaiseLowerMotor.Set(speed);
        }
};

// Package the setSpeed method for use as a Command
frc2::CommandPtr ElevatorSubsystem::runSetSpeed(double speed) {
    return this->Run(
        [this, speed=std::move(speed)] { this->setSpeed(speed); }
    );
};