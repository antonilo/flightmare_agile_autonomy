#include <rpgq_command_bridge/rpgq_command_bridge.h>

using namespace RPGQ;

int main(int argc, char **argv)
{
  bool client = false;
  if (argc < 2)
  {
    ros::init(argc, argv, "command_bridge_server");
    ROS_INFO("[/command_bridge_server] No argument given, initializing as server.");
  }
  else
  {
    std::string str_server = "server", str_client = "client";
    if (str_server.compare(argv[1]) == 0)
    {
      client = false;
      ros::init(argc, argv, "command_bridge_server");
    }
    else if (str_client.compare(argv[1]) == 0)
    {
      client = true;
      ros::init(argc, argv, "command_bridge_client");
    }
    else
    {
      client = false;
      ros::init(argc, argv, "command_bridge_server");
      ROS_INFO("[/command_bridge_server] Invalid argument, initializing as server.");
    }
  }

  if (ros::master::check())
  {
    if (client)
    {
      CommandBridge::CommandBridgeClient commandBridgeClient;
      ros::spin();
    }
    else
    {
      CommandBridge::CommandBridgeServer commandBridgeServer;
      ros::spin();
    }
    return 0;
  }
  else
  {
    std::printf("ROS core not running.\n");
    return 0;
  }
}
