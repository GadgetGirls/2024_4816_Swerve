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
    double elevator_hold_speed = 0.2;
    // Going up, speed is positive, going down speed is negative
    if (speed > 0){
        // Limit switches return false when closed/triggered.
        if (m_upperLimitSwitch.Get() == false){
                m_elevatorRaiseLowerMotor.Set(0);
        } else {
                m_elevatorRaiseLowerMotor.Set(speed);
        }
    } else {
        if (m_lowerLimitSwitch.Get() == false){
                m_elevatorRaiseLowerMotor.Set(0);
        } else {
                m_elevatorRaiseLowerMotor.Set(speed);
        }   
    }
};


void ElevatorSubsystem::runForTime(units::second_t seconds, double speed){
    frc::Timer timer = frc::Timer();
    timer.Start();
    while (!timer.HasElapsed(seconds)){
        setSpeed(speed);
    }
    setSpeed(0);
}

void ElevatorSubsystem::autoRaise(){
    // Raise the elevator for configured seconds
    m_elevatorTimer.Start();
    while(!m_elevatorTimer.HasElapsed(m_raiseTime)){
      setSpeed(0.75);
    }
    setSpeed(0);
}


// Package the setSpeed method for use as a Command
frc2::CommandPtr ElevatorSubsystem::runSetSpeed(double speed) {
    return this->Run(
        [this, speed=std::move(speed)] { this->setSpeed(speed); }
    );
};