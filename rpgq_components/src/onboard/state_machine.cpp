#include <rpgq_components/onboard/state_machine.h>

using namespace RPGQ::Onboard;

// constructor
StateMachine::StateMachine(void)
{
    // onboard state variables
    state_ = OnboardState::DISARMED;
    lastState_ = OnboardState::DISARMED;

    // command variables and functions
    newCommand_ = false;
    command_ = rpgq_msgs::CreateIdleCommand(RPGQ::QUAD_INVALID);
}


// public update function
void StateMachine::Update(void)
{
    // update LEDs

    // store current state
    lastState_ = state_;

    /*********************
     * run state machine *
     *********************/

    // if there's an internal error, stop here
    if (state_ == OnboardState::INTERNAL_ERROR) return;

    // update command
    if (UpdateCommand())
    {
        // reset timer since last command
        //timerLastCommand_Reset();

        // change state if allowed and required
        switch(state_)
        {
        case OnboardState::DISARMED:
        case OnboardState::COMMAND_LOST:
            // it is only allowed to arm the vehicle
            if (GetCommandType(command_) == rpgq_msgs::CommandType::ARM)
            {
                state_ = OnboardState::ARMED;
                return;
            }
            break;

        case OnboardState::LOW_BATTERY:
            // it is only allowed to disarm or shutdown the vehicle
            if (GetCommandType(command_) == rpgq_msgs::CommandType::DISARM)
            {
                state_ = OnboardState::DISARMED;
                return;
            }
            if (GetCommandType(command_) == rpgq_msgs::CommandType::SHUTDOWN)
            {
                state_ = OnboardState::SHUTDOWN;
            }
            break;

        case OnboardState::ARMED:
            // it is allowed to shut down
            if (GetCommandType(command_) == rpgq_msgs::CommandType::SHUTDOWN)
            {
                state_ = OnboardState::SHUTDOWN;
            }
            // do not break here!

        default:
            state_ = OnboardStateFromCommand(command_);
            return;
        }
    }
    else
    {
        // no new command available

        // check if continuous commands are required
        if (RequiresContinuousCommands(state_))
        {
            /*if (timerLastCommand_.ElapsedSeconds() > TODO)
            {
                // lost commands
                state_ = OnboardState::COMMAND_LOST;
                return;
            }*/
        }
        else
        {
            // don't do anything
            return;
        } 
    }
}


// public callback functions (simulation only)
void StateMachine::CommandCallback(const rpgq_msgs::Command &command)
{
    newCommand_ = true;
    command_ = command;
}


// private onboard state functions
bool StateMachine::RequiresContinuousCommands(const OnboardState& state)
{
    switch (state)
    {
    // state that don't require continuous commands
    case OnboardState::ARMED:
    case OnboardState::SHUTDOWN:
    case OnboardState::DISARMED:
    case OnboardState::COMMAND_LOST:
    case OnboardState::LOW_BATTERY:
    case OnboardState::INTERNAL_ERROR:
        return false;

    // by default all other states return continous commands
    default:
        return true;
    }
}

OnboardState StateMachine::OnboardStateFromCommand(const rpgq_msgs::Command& cmd)
{
    switch (GetCommandType(cmd))
    {
    // 0 - 9: general commands
    case rpgq_msgs::CommandType::IDLE:
        return OnboardState::ARMED;

    case rpgq_msgs::CommandType::ACTUATOR_IDLE:
        return OnboardState::ACTUATOR_IDLE;

    case rpgq_msgs::CommandType::ARM:
        return OnboardState::ARMED;

    case rpgq_msgs::CommandType::DISARM:
        return OnboardState::DISARMED;

    // 10 - 29: flying commands
    case rpgq_msgs::CommandType::THRUST_ANGULAR_VELOCITY:
        return OnboardState::THRUST_ANGULAR_RATE;

    // other commands
    default:
        return OnboardState::INTERNAL_ERROR;
    }
}


// private command functions
bool StateMachine::UpdateCommand(void)
{
    if (newCommand_)
    {
        newCommand_ = false;
        return true;
    }
    else
    {
        return false;
    }
}