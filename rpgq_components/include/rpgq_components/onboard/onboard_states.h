#pragma once

namespace RPGQ
{
    namespace Onboard
    {
        // onboard states
        enum OnboardState
        {
            // 0 - 9: on ground
            ARMED = 0,
            ACTUATOR_IDLE = 1,
            SHUTDOWN = 2,


            // 10 - 29: flying
            THRUST_ANGULAR_RATE = 10,

            // 40 - 49: emergency
            DISARMED = 40,
            COMMAND_LOST = 41,
            LOW_BATTERY = 42,
            INTERNAL_ERROR = 43
        };

        // onboard state to string conversion
        static const char* StateName(OnboardState in)
        {
            switch(in)
            {
            // 0 - 9: on ground
            case ARMED:
                return "Armed";
            case ACTUATOR_IDLE:
                return "Actuator idle";
            case SHUTDOWN:
                return "Shutdown";
            
            // 10 - 29: flying
            case THRUST_ANGULAR_RATE:
                return "Thrust and angular rate control";

            // 40 - 49: emergency
            case DISARMED:
                return "Disarmed";
            case COMMAND_LOST:
                return "Command lost";
            case LOW_BATTERY:
                return "Low battery";
            case INTERNAL_ERROR:
                return "Internal error";

            // unknown state
            default:
                return "Unknown!";
            }
        };
    } // namespace Onboard
} // namespace RPGQ