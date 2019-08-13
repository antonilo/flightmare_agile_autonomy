#include <rpgq_command_bridge/rpgq_command_bridge.h>

// rpgq rpgq_common
#include <rpgq_common/network/cobs.h>
#include <rpgq_common/network/crc.h>
#include <rpgq_common/time/timer.h>

// standard library
#include <poll.h>

#define PACKET_HEADER_SIZE            2
#define START_SYMBOL_BYTE             0
#define PACKET_LENGTH_BYTE            1
#define BUFFER_LENGTH                 256

using namespace RPGQ;
using namespace RPGQ::CommandBridge;

/*************************************************************
 * Command Bridge Server 
 ************************************************************/

// constructor & destructor
CommandBridgeServer::CommandBridgeServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
{
	// general variables
	nh_ = nh;
	pnh_ = pnh;
	armed_ = false;

	// initialize serial port
	std::string port;
	int baud;
	pnh_.getParam("port", port);
	pnh_.getParam("baud_rate", baud);

	if (!serialPort_.Initialize(port, baud))
	{
		ROS_INFO("[%s] Unable to open serial port %s.", pnh_.getNamespace().c_str(), port.c_str());
		ros::shutdown();
		return;
	}

	// set up subscribers
	armBridgeSub_ = nh_.subscribe("command_bridge_server/arm", 1, &CommandBridgeServer::ArmCallback, this);
	commandSub_ = nh_.subscribe("command_bridge_server/command", 10, &CommandBridgeServer::CommandCallback, this);

	// call main thread
	ROS_INFO("[%s] Set up done. Executing main thread...", pnh_.getNamespace().c_str());
	shouldExit_ = false;
	std::thread threadHandle(&CommandBridgeServer::MainThread, this);
	threadHandle.detach();
}

CommandBridgeServer::~CommandBridgeServer(void)
{
	shouldExit_ = true;
	
	std::unique_lock<std::mutex> lck(commandQueueMutex_);
	commandQueueCv_.notify_all();
}


// main command bridge server functions
void CommandBridgeServer::MainThread(void)
{
	// reserve buffer for data
	uint8_t buffer[BUFFER_LENGTH];

	// main loop
	Timer timer;
	while(!shouldExit_)
	{
		// reset time
		timer.Reset();

		// generate command set message
		rpgq_msgs::CommandSet msg;
		msg.length = 0;

		// process command queue
		commandQueueMutex_.lock();
		for (const auto command : commandQueue_)
		{
			if (armed_)
			{
				rpgq_msgs::AddCommandToCommandSet(command.second, msg);
			}
			else
			{
				rpgq_msgs::Command disarmCmd = rpgq_msgs::CreateDisarmCommand(command.first);
				rpgq_msgs::AddCommandToCommandSet(disarmCmd, msg);
			}
		}
		commandQueue_.clear();
		commandQueueMutex_.unlock();

		// send out commands
		if (msg.length > 0)
		{
			// serialize message
			uint8_t serializedMessageLength = ros::serialization::serializationLength(msg);
			uint8_t serializedMessage[serializedMessageLength + crc_.GetChecksumLength()];
			ros::serialization::OStream stream(serializedMessage, serializedMessageLength);
			ros::serialization::serialize(stream, msg);

			// add checksum
			const uint16_t checksum = crc_.GetChecksum(serializedMessage, serializedMessageLength);
			std::memcpy(&serializedMessage[serializedMessageLength], &checksum, sizeof(uint16_t));

			// insert start symbol
			buffer[START_SYMBOL_BYTE] = COBS::COBS_START_SYMBOL;

			// encode message
			uint8_t ret; size_t length;
			std::tie(ret, length) = COBSEncode(serializedMessage, serializedMessageLength + crc_.GetChecksumLength(),
														&buffer[PACKET_HEADER_SIZE], BUFFER_LENGTH - PACKET_HEADER_SIZE);
			if (ret != COBS::COBS_ENCODE_OK)
			{
				ROS_WARN("[%s] COBS encoding error: 0x%02x", pnh_.getNamespace().c_str(), (unsigned) ret);
				continue;
			}

			// insert packet length
			buffer[PACKET_LENGTH_BYTE] = (uint8_t) length;

			// send out message
			if (length + PACKET_HEADER_SIZE > MAX_COMMAND_SET_SIZE)
			{
				ROS_WARN("[%s] Cannot send command set message! Message is too long (%u instead of maximum %u bytes).", pnh_.getNamespace().c_str(), serializedMessageLength, MAX_COMMAND_SET_SIZE);
			}
			else
			{
				serialPort_.Write(buffer, length + PACKET_HEADER_SIZE);
			}

			// sleep
			double secsToSleep = 1.0/60.0 - timer.ElapsedSeconds(); // TODO update rate
			if (secsToSleep > 0.0)
			{
				usleep(secsToSleep*1e6);
			}
		}
		else
		{
			std::unique_lock<std::mutex> lck(commandQueueMutex_);
			commandQueueCv_.wait(lck, [&]() {return !commandQueue_.empty();});
		}
	}
}


// subscriber functions
void CommandBridgeServer::ArmCallback(const std_msgs::BoolConstPtr &msg)
{
	if (msg->data != armed_)
	{
		armed_ = msg->data;

		if (armed_)
		{
			ROS_INFO("[%s] Armed!", pnh_.getNamespace().c_str());
		}
		else
		{
			ROS_INFO("[%s] Disarmed!", pnh_.getNamespace().c_str());
		}
	}
}

void CommandBridgeServer::CommandCallback(const rpgq_msgs::Command::ConstPtr& msg)
{
	std::unique_lock<std::mutex> lck(commandQueueMutex_);

	std::unordered_map<uint8_t, rpgq_msgs::Command>::const_iterator idx = commandQueue_.find(msg->id);
	if (idx != commandQueue_.end())
	{
		ROS_WARN("[%s] Receiving multiple commands for ID %d. Ignorind old commands.", pnh_.getNamespace().c_str(), msg->id);
		commandQueue_[msg->id] = *msg;
	}
	else
	{
		commandQueue_.emplace(msg->id, *msg);
	}

	commandQueueCv_.notify_all();
}

/*************************************************************
 * Command Bridge Client 
 ************************************************************/

// constructor & destructor
CommandBridgeClient::CommandBridgeClient(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
{
	// general variables
	nh_ = nh;
	pnh_ = pnh;

	// main command bridge client variables
	int id;
	pnh_.getParam("id", id);
	id_ = (uint8_t) id;

	// initialize serial port
	std::string port;
	int baud;
	pnh_.getParam("port", port);
	pnh_.getParam("baud_rate", baud);

	if (!serialPort_.Initialize(port, baud))
	{
		ROS_INFO("[%s] Unable to open serial port %s.", pnh_.getNamespace().c_str(), port.c_str());
		ros::shutdown();
		return;
	}

	// publisher variables
	commandPub_ = nh_.advertise<rpgq_msgs::Command>("command_bridge_client/command", 1);

	// auxliary variables
	parsingState_ = ParsingState::START_SYMBOL;

	// call main thread
	ROS_INFO("[%s] Set up done. Executing main thread...", pnh_.getNamespace().c_str());
	shouldExit_ = false;
	std::thread threadHandle(&CommandBridgeClient::MainThread, this);
	threadHandle.detach();
}

CommandBridgeClient::~CommandBridgeClient(void)
{
	shouldExit_ = true;
}


// main command bridge client functions
void CommandBridgeClient::MainThread(void)
{
	// set up buffer, decoded packet
	int lengthRead;
	uint8_t buffer[BUFFER_LENGTH];
	uint8_t decodedPacket[MAX_COMMAND_SET_SIZE - PACKET_HEADER_SIZE];
	size_t decodedPacketLength = 0;

	// main loop
	struct pollfd fds;
	fds.fd = serialPort_.GetFileDescriptor();
	fds.events = POLLIN;
	while(!shouldExit_)
	{
		int ret = poll(&fds, 1, 1000);    // timeout 1000ms
		if (ret == 0)
		{
			// timeout
			continue;
		}

		// read from serial port
		lengthRead = serialPort_.Read(buffer, BUFFER_LENGTH);
		while (lengthRead > 0)
		{
			if (shouldExit_)
			{
			break;
			}

			for (int i = 0; i < lengthRead; i++)
			{
				if (ParseIncomingByte(buffer[i], decodedPacket, decodedPacketLength))
				{
					// deserialize message
					rpgq_msgs::CommandSet cmdSetMsg;
					ros::serialization::IStream stream(decodedPacket, decodedPacketLength);
					ros::serialization::deserialize(stream, cmdSetMsg);

					rpgq_msgs::Command cmdMsg;
					if (GetCommandFromCommandSet(cmdSetMsg, id_, cmdMsg))
					{
						// publish command
						commandPub_.publish(cmdMsg);
					}
				}
			}

			lengthRead = serialPort_.Read(buffer, BUFFER_LENGTH);
		}
	}
}


// auxiliary functions
bool CommandBridgeClient::ParseIncomingByte(uint8_t currentByte, uint8_t* decodedPacket, size_t &decodedPacketLength)
{
	static uint8_t encodedPacket[MAX_COMMAND_SET_SIZE - PACKET_HEADER_SIZE];
	static uint8_t encodedPacketLength = 0;
	static uint8_t idx = 0;

	if (currentByte == COBS::COBS_START_SYMBOL)
	{
		parsingState_ = ParsingState::LENGTH;
		encodedPacketLength = 0;
		idx = 0;
	}
	else if (parsingState_ == ParsingState::LENGTH)
	{
		// check packet length
		if (currentByte > MAX_COMMAND_SET_SIZE - PACKET_HEADER_SIZE|| currentByte < crc_.GetChecksumLength())
		{
			parsingState_ = ParsingState::START_SYMBOL;
		}
		else
		{
			encodedPacketLength = currentByte;
			parsingState_ = ParsingState::PAYLOAD;
		}
	}
	else if (parsingState_ == ParsingState::PAYLOAD)
	{
		encodedPacket[idx++] = currentByte;

		// reached end, perform check checksum
		if (idx >= encodedPacketLength)
		{
			uint8_t ret;
			std::tie(ret, decodedPacketLength) = COBSDecode(encodedPacket, encodedPacketLength, decodedPacket, MAX_COMMAND_SET_SIZE);
			if (ret != COBS::COBS_DECODE_OK)
			{
				ROS_WARN("[%s] COBS decoding error: 0x%02x", pnh_.getNamespace().c_str(), (unsigned) ret);
				return false;
			}

			decodedPacketLength -= crc_.GetChecksumLength();
			const uint16_t checksum = crc_.GetChecksum(decodedPacket, decodedPacketLength);
			if (std::memcmp(&checksum, &decodedPacket[decodedPacketLength], crc_.GetChecksumLength()) != 0)
			{
				ROS_WARN("[%s] Checksum error: 0x%02x 0x%02x instead of 0x%02x 0x%02x", pnh_.getNamespace().c_str(),
					(unsigned) (checksum & 0xff), (unsigned) ((checksum >> 8) & 0xff), (unsigned) decodedPacket[decodedPacketLength], (unsigned) decodedPacket[decodedPacketLength + 1]);
				return false;
			}
			else
			{
				return true;
			}
		}
	}

	return false;
}