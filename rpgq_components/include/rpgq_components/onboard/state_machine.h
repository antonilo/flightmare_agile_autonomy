#pragma once

// rpgq rpgq_common
#include <rpgq_common/quadrotor_id.h>
#include <rpgq_common/msgs/command_msg_helper.h>

// rpgq rpgq_components
#include <rpgq_components/onboard/onboard_states.h>

// rpgq rpgq_msgs
#include <rpgq_msgs/Command.h>

// ros
#include <ros/ros.h>

namespace RPGQ
{
    namespace Onboard
    {
        class StateMachine
        {
        public:
            // constructor & destructor
            StateMachine(void);
            ~StateMachine(void) {};

            // public update function
            void Update(void);

            // public callback function (simulation only)
            void CommandCallback(const rpgq_msgs::Command &command);

            // public print functions
            void PrintStatus(void) const;

            // public get functions
            OnboardState GetState(void) const {return state_;};
            rpgq_msgs::Command GetCurrentCommand(void) const {return command_;};

            // public set functions
            void SetInternalError(void)
            {
                lastState_ = state_;
                state_ = OnboardState::INTERNAL_ERROR;
            };


        private:
            // onboard state variables and functions
            OnboardState state_;
            OnboardState lastState_;
            static bool RequiresContinuousCommands(const OnboardState& state);
            static OnboardState OnboardStateFromCommand(const rpgq_msgs::Command& cmd);

            // command variables and functions
            bool newCommand_ = true;
            rpgq_msgs::Command command_;
            bool UpdateCommand(void);

            // power sensor

            // LEDs

            //updatesSinceLastCommand_;
        };
    }
}