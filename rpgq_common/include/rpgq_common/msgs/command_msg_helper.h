#pragma once

// rpgq rpgq_common
#include <rpgq_common/quadrotor_id.h>

// rpgq messages
#include <rpgq_msgs/Command.h>
#include <rpgq_msgs/CommandSet.h>

// standard library
#include <cstdint>
#include <unordered_map>
#include <vector>

// others
#include <eigen3/Eigen/Dense>

namespace rpgq_msgs
{
    // typedefs
    typedef uint8_t CommandID;
    typedef std::unordered_map<CommandID, Command> CommandQueue;

    // command types
    enum CommandType
    {
        // 0 - 9: general commands
        IDLE = 0,
        ACTUATOR_IDLE = 1,
        ARM = 2,
        DISARM = 3,
        SHUTDOWN = 4,

        // 10 -29: flying commands
        THRUST_ANGULAR_VELOCITY = 10,

        // invalid commands
        INVALID = 255
    };

    // create command functions
    Command CreateIdleCommand(RPGQ::QuadrotorID quadrotorID);
    Command CreateActuatorIdleCommand(RPGQ::QuadrotorID quadrotorID);
    Command CreateArmCommand(RPGQ::QuadrotorID quadrotorID);
    Command CreateDisarmCommand(RPGQ::QuadrotorID quadrotorID);
    Command CreateShutdownCommand(RPGQ::QuadrotorID quadrotorID);
    Command CreateThrustAngularVelocityCommand(RPGQ::QuadrotorID quadrotorID, double thrust, Eigen::Vector3d &omega);

    // helper functions
    void AddFloatToPayload(std::vector<uint8_t> &payload, float value);
    void AddCommandToCommandSet(const Command &cmd, CommandSet &cmdSet);
    CommandType GetCommandType(const Command &cmd);
    int GetCommandPayloadLength(CommandType type);
    int GetCommandSetPayloadLength(const CommandQueue* commandQueue);
    bool GetCommandFromCommandSet(const CommandSet &cmdSet, const RPGQ::QuadrotorID quadID, Command &cmd);

    bool GetFloatFromPayloadAt(const std::vector<uint8_t> &payload, const unsigned i, float &val);
    bool GetThrustAngularVelocityFromCommand(const Command& cmd, double &thrust, Eigen::Vector3d &omega);


} // namespace rpgq_msgs