#pragma once

// ros
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// rpgq rpgq_common
#include <rpgq_common/msgs/command_msg_helper.h>
#include <rpgq_common/serial_port/serial_port.h>
#include <rpgq_common/network/crc.h>
#include <rpgq_common/quadrotor_id.h>

// rpgq rpgq_msgs
#include <rpgq_msgs/Command.h>
#include <rpgq_msgs/CommandSet.h>

// standard library
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>


namespace RPGQ
{
    namespace CommandBridge
    {
        // constants
        const uint8_t MAX_COMMAND_SET_SIZE = 239; // maximum size of a packet transmitted with a LAIRD RM024


        // server
        class CommandBridgeServer
        {
        public:
            // constructor & destructor
            CommandBridgeServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
            CommandBridgeServer(void):
                CommandBridgeServer(ros::NodeHandle(), ros::NodeHandle("~"))
            {};
            ~CommandBridgeServer(void);

        private:
            // general variables
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            bool armed_;
            bool shouldExit_;

            // main command bridge server variables and functions
            rpgq_msgs::CommandQueue commandQueue_;
            std::mutex commandQueueMutex_;
            std::condition_variable commandQueueCv_;
            CRC16_CCITT_Table crc_;
            void MainThread(void);

            // serial port variables
            SerialPort serialPort_;

            // subscriber variables and functions
            ros::Subscriber armBridgeSub_;
            ros::Subscriber commandSub_;
            void ArmCallback(const std_msgs::BoolConstPtr &msg);
            void CommandCallback(const rpgq_msgs::Command::ConstPtr& msg);
        };


        // client(s)
        class CommandBridgeClient
        {
        public:
            // constructor & destructor
            CommandBridgeClient(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
            CommandBridgeClient(void):
            CommandBridgeClient(ros::NodeHandle(), ros::NodeHandle("~"))
            {};
            ~CommandBridgeClient(void);

        private:
            // general variables
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            bool shouldExit_;

            // main command bridge client variables and functions
            uint8_t id_;
            CRC16_CCITT_Table crc_;
            void MainThread(void);

            // serial port variables
            SerialPort serialPort_;

            // publisher variables
            ros::Publisher commandPub_;

            // auxiliary variables & functions
            enum ParsingState
            {
                START_SYMBOL = 0,
                LENGTH = 1,
                PAYLOAD = 2
            } parsingState_;
            bool ParseIncomingByte(uint8_t currentByte, uint8_t* decodedPacket, size_t &decodedPacketLength);
        };

    } // namespace CommandBridge
} // namespace RPGQ