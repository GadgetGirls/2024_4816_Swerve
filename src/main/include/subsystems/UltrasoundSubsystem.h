// Declare a Subsystem to operate the 2025 ultrasound range finder

#pragma once

#include <frc/AnalogInput.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Ultrasonic.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

class UltrasoundSubsystem : public frc2::SubsystemBase {
    public:
        // Constructor
        UltrasoundSubsystem();
        
        // Destructor
        ~UltrasoundSubsystem();

        // Actions to take regularly as the robot runs
        void Periodic() override;
        
        /***
         * Methods that directly do a thing
         ***/
        // Get the range reported by the sensor
         units::length::meter_t getRange();
        
        // Are we close enough to something to touch it?
        bool areWeCloseEnough();

        /***
         * Methods that return a CommandPtr to a thing that does the thing
         ***/
        
    private:
        /***
         * Internal data to the subsystem
         */
        static constexpr int kUltrasonicPingPort = 1;
        static constexpr int kUltrasonicEchoPort = 0;
        frc::Ultrasonic m_ultrasonic{kUltrasonicPingPort, kUltrasonicEchoPort};
        // frc::AnalogInput ultrasonic_input(0);

};