#include <rpgq_common/msgs/command_msg_helper.h>

// standard library
#include <cstring>

namespace rpgq_msgs
{
    // command creation functions
    Command CreateIdleCommand(RPGQ::QuadrotorID quadrotorID)
    {
        Command cmd;
        cmd.id = quadrotorID;
        cmd.type = CommandType::IDLE;

        return cmd;
    };

    Command CreateActuatorIdleCommand(RPGQ::QuadrotorID quadrotorID)
    {
        Command cmd;
        cmd.id = quadrotorID;
        cmd.type = CommandType::ACTUATOR_IDLE;

        return cmd; 
    };

    Command CreateArmCommand(RPGQ::QuadrotorID quadrotorID)
    {
        Command cmd;
        cmd.id = quadrotorID;
        cmd.type = CommandType::ARM;

        return cmd;
    };

    Command CreateDisarmCommand(RPGQ::QuadrotorID quadrotorID)
    {
        Command cmd;
        cmd.id = quadrotorID;
        cmd.type = CommandType::DISARM;

        return cmd;
    };

    Command CreateShutdownCommand(RPGQ::QuadrotorID quadrotorID)
    {
        Command cmd;
        cmd.id = quadrotorID;
        cmd.type = CommandType::SHUTDOWN;

        return cmd;  
    }

    Command CreateThrustAngularVelocityCommand(RPGQ::QuadrotorID quadrotorID, double thrust, Eigen::Vector3d &omega)
    {
        Command cmd;
        cmd.id = quadrotorID;
        cmd.type = CommandType::THRUST_ANGULAR_VELOCITY;
        AddFloatToPayload(cmd.payload, thrust);
        AddFloatToPayload(cmd.payload, omega.x());
        AddFloatToPayload(cmd.payload, omega.y());
        AddFloatToPayload(cmd.payload, omega.z());

        return cmd;
    };

    // helper functions
    void AddFloatToPayload(std::vector<uint8_t> &payload, float value)
    {
        for (uint8_t i = 0; i < sizeof(float); i++)
        {
            payload.push_back(*((uint8_t *) &value + i));
        }
    };

    void AddCommandToCommandSet(const Command &cmd, CommandSet &cmdSet)
    {
        cmdSet.id.push_back(cmd.id);
        cmdSet.type.push_back(cmd.type);
        cmdSet.payload.insert(cmdSet.payload.end(), cmd.payload.begin(), cmd.payload.end());
        cmdSet.length++;

        // TODO what if command already exists
    };

    CommandType GetCommandType(const Command &cmd)
    {
        return (CommandType) cmd.type;
    }

    int GetCommandPayloadLength(CommandType type)
    {
        switch(type)
        {
            case ARM:
            case DISARM:
                return 0;

            case THRUST_ANGULAR_VELOCITY:
                return 16; // 4x sizeof(float)               

            default:
                return -1;
        };
    };

    int GetCommandSetPayloadLength(const CommandQueue* commandQueue)
    {
        int payloadLength = 0;

        for (auto command : *commandQueue)
        {
            payloadLength += GetCommandPayloadLength((CommandType) command.second.type);
        }

        return payloadLength;
    };

    bool GetCommandFromCommandSet(const CommandSet &cmdSet, const RPGQ::QuadrotorID quadID, Command & cmd)
    {
        uint8_t payloadIdx = 0;
        for (uint8_t j = 0; j < cmdSet.length; j++)
        {
            if (cmdSet.id.at(j) == quadID)
            {
                // extract command message
                cmd.header = cmdSet.header;
                cmd.id = quadID;
                cmd.type = cmdSet.type.at(j);
                uint8_t payloadIdxEnd = payloadIdx + GetCommandPayloadLength((rpgq_msgs::CommandType) cmdSet.type.at(j));
                cmd.payload.clear();
                cmd.payload.insert(cmd.payload.end(), cmdSet.payload.begin() + payloadIdx, cmdSet.payload.begin() + payloadIdxEnd);

                return true;
            }
            else
            {
                payloadIdx += GetCommandPayloadLength((rpgq_msgs::CommandType) cmdSet.type.at(j));
            }
        }

	    return false;
    };

    bool GetFloatFromPayloadAt(const std::vector<uint8_t> &payload, const unsigned i, float &val)
    {
        if (!(i + sizeof(float) > payload.size()))
        {
            for (uint8_t j = 0; j < sizeof(float); j++)
            {
                *((uint8_t *) &val + j) = payload.at(i + j);
            }

            return true;
        }
        else
        {
            return false;
        }
    };

    bool GetThrustAngularVelocityFromCommand(const Command& cmd, double &thrust, Eigen::Vector3d &omega)
    {
        if (cmd.type == CommandType::THRUST_ANGULAR_VELOCITY)
        {
            uint8_t cnt(0);

            bool ret(true);
            float val;
            ret &= GetFloatFromPayloadAt(cmd.payload, cnt++*sizeof(float), val); thrust = (double) val;
            ret &= GetFloatFromPayloadAt(cmd.payload, cnt++*sizeof(float), val); omega.x() = (double) val;
            ret &= GetFloatFromPayloadAt(cmd.payload, cnt++*sizeof(float), val); omega.y() = (double) val;
            ret &= GetFloatFromPayloadAt(cmd.payload, cnt++*sizeof(float), val); omega.z() = (double) val;

            return ret;
        }
        else
        {
            return false;
        }
    }


} // namespace rpgq_msgs