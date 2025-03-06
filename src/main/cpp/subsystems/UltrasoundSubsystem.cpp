#include "subsystems/UltrasoundSubsystem.h"

// Define our ElevatorSubsystem Constructor
UltrasoundSubsystem::UltrasoundSubsystem(){};

// The Linker says we have to define a non-virtual destructor function
UltrasoundSubsystem::~UltrasoundSubsystem(){
    // Could optionally explicitly delete x here
};

// Do periodic subsystem things here. 
void UltrasoundSubsystem::Periodic(){
    // Update reading?
};

// Get the range reported by the sensor
units::length::meter_t UltrasoundSubsystem::getRange(){
    return m_ultrasonic.GetRange();
};
        
// Are we close enough to something to touch it?
bool UltrasoundSubsystem::areWeCloseEnough(){
    return getRange() < 100_cm;
};

